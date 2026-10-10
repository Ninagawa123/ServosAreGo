#include <Arduino.h>
#include "servo_bus.h"
#include "servo_select.h"

namespace feetech
{

// 基本型定義
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;

// ピン定義
#define PIN_EN 33

// 命令定義
#define INST_SYNC_WRITE 0x83 // 同期書き込み命令（SyncWritePos用）
#define INST_REG_WRITE 0x04  // 非同期書き込み命令
#define INST_REG_ACTION 0x05 // 非同期実行命令

// SMS/STSレジスタ定義
#define SMS_STS_ACC 41
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_LOCK 55

// SCSレジスタ定義
#define SCS_GOAL_POSITION 42
#define SCS_LOCK 48

// 最大パケットサイズ定義
#define MAX_PACKET_SIZE 256

// 一斉制御リストの最大数（有効ID 1-253）
#define MAX_SYNC_SERVOS 253

// 位置値の定数定義
#define POSITION_MIN 0
#define STS_POSITION_MAX 4095
#define STS_ANGLE_RANGE 360.0
#define SCS_POSITION_MAX 1023
#define SCS_ANGLE_RANGE 300.0 // SCS0009等の可動範囲（機種により異なる）

// サーボ系列定義（アドレス2のEND値: 0=リトルエンディアン(STS), 1=ビッグエンディアン(SCS)）
#define SERVO_TYPE_STS 0
#define SERVO_TYPE_SCS 1

// サーボレジスタアドレス定義
#define REG_FIRMWARE_MAJOR 0
#define REG_END 2
#define REG_ID 5
#define REG_BAUD_RATE 6
#define REG_CURRENT_POSITION 56
#define REG_VOLTAGE 62

// サーボ検索のデフォルト最大ID（[f]で空欄リターン時）
#define DEFAULT_SCAN_MAX_ID 20

// コマンド定義
#define CMD_PING 1
#define CMD_READ 2
#define CMD_WRITE 3

// 電圧範囲定義
#define VOLTAGE_MIN 50  // 5.0V
#define VOLTAGE_MAX 140 // 14.0V

// STSサーボ標準通信速度（インデックス = ボーレートレジスタ値）
const unsigned long STS_BAUD_RATES[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
#define STS_BAUD_COUNT (sizeof(STS_BAUD_RATES) / sizeof(STS_BAUD_RATES[0]))

// 既知のサーボ型番（アドレス3-4の値、FEETECHマニュアル準拠のLeRobot定義より）
struct ServoModel
{
  u16 number;
  const char *name;
};
const ServoModel STS_MODELS[] = {
    {777, "STS3215"},
    {2825, "STS3250"},
    {11272, "SM8512BL"},
    {1284, "SCS0009"},
};
#define STS_MODEL_COUNT (sizeof(STS_MODELS) / sizeof(STS_MODELS[0]))

// グローバル変数
HardwareSerial *servoSerial = &Serial1; // 使用するシリアルポート
// u8 responseLevel = 0;                   // 応答不要（送信専用モード）
// unsigned long ioTimeout = 100;
u8 targetServoID = 1;  // ターゲットサーボID
u8 syncServoIDs[MAX_SYNC_SERVOS] = {1, 2, 3, 4, 5, 6}; // 一斉制御するサーボIDリスト（とびとびIDも可）
u8 syncServoCount = 6;                                  // リスト内のサーボ数
#define DEFAULT_BAUD_INDEX 0 // 1Mbps（工場出荷値）
u8 servoBaudIndex = DEFAULT_BAUD_INDEX; // ESP32側の現在の通信速度インデックス
u8 servoType = SERVO_TYPE_STS; // 現在のサーボ系列（起動時のサーボ種類選択/[f]自動判定で変更）

// サーボ系列名
const char *servoTypeName(u8 type)
{
  return type == SERVO_TYPE_SCS ? "SCS" : "STS";
}

// 系列別のロックフラグアドレス
u8 regLock()
{
  return servoType == SERVO_TYPE_SCS ? SCS_LOCK : SMS_STS_LOCK;
}

// 系列別の位置最大値
u16 positionMax()
{
  return servoType == SERVO_TYPE_SCS ? SCS_POSITION_MAX : STS_POSITION_MAX;
}

// 系列別の可動角度範囲
float angleRange()
{
  return servoType == SERVO_TYPE_SCS ? SCS_ANGLE_RANGE : STS_ANGLE_RANGE;
}

// 系列別の中央位置（0度）
u16 positionCenter()
{
  return (positionMax() + 1) / 2;
}

// 位置値から角度への変換（中央 = 0度）
float positionToAngle(u16 position)
{
  return ((float)position - positionCenter()) / (positionMax() + 1) * angleRange();
}

// 2バイト値のデコード（STS: リトルエンディアン, SCS: ビッグエンディアン）
u16 decodeU16(const u8 *data, u8 type)
{
  return type == SERVO_TYPE_SCS ? (u16)((data[0] << 8) | data[1]) : (u16)((data[1] << 8) | data[0]);
}

// 2バイト値のエンコード（STS: リトルエンディアン, SCS: ビッグエンディアン）
void encodeU16(u16 value, u8 *data)
{
  if (servoType == SERVO_TYPE_SCS)
  {
    data[0] = value >> 8;
    data[1] = value & 0xff;
  }
  else
  {
    data[0] = value & 0xff;
    data[1] = value >> 8;
  }
}

// シリアル送信関数
void writeServo(u8 data)
{
  servoSerial->write(data);
}

void writeServo(u8 *data, int length)
{
  servoSerial->write(data, length);
}

void flushServo()
{
  servoSerial->flush();
  delay(5); // 送信完了確保
}

// 受信バッファフラッシュ
void flushReceiveBuffer()
{
  while (servoSerial->read() != -1)
    ;
}

// チェックサム計算
u8 calcChecksum(u8 *data, int length)
{
  u8 checksum = 0;
  for (int i = 2; i < length - 1; i++)
  {
    checksum += data[i];
  }
  return ~checksum;
}

// 半二重通信用コマンド送信（読み取り用）
void sendCommand(u8 *command, int length)
{
  // 受信バッファクリア
  flushReceiveBuffer();

  // 送信 → 送信完了後すぐ受信モードに切り替え → 送信エコーを読み捨て
  busDirTx();
  servoSerial->write(command, length);
  busEndSendToRx();
  busSkipEcho(command, length, 10);
}

// 応答受信
bool receiveResponse(u8 *response, int expectedLength, unsigned long timeout)
{
  unsigned long startTime = millis();
  int bytesReceived = 0;

  while (bytesReceived < expectedLength && (millis() - startTime) < timeout)
  {
    if (servoSerial->available())
    {
      response[bytesReceived] = servoSerial->read();
      bytesReceived++;
    }
  }

  // 受信完了後、送信専用モードに戻す（元の設定に復帰）
  busDirTx();

  return bytesReceived == expectedLength;
}

// SyncWrite位置パケット構築（系列別フォーマット）。戻り値はパケット長（0はエラー）
// STS: アドレス41から7バイト（ACC + Position + Time + Speed）
// SCS: アドレス42から6バイト（Position + Time + Speed、ACCなし）
u16 buildSyncWritePosPacket(u8 *packet, u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
  const bool isSCS = (servoType == SERVO_TYPE_SCS);
  const u8 dataPerServo = isSCS ? 6 : 7;
  const u8 startAddress = isSCS ? SCS_GOAL_POSITION : SMS_STS_ACC;

  // パケット全体のサイズ計算（ヘッダ7バイト + データ部 + チェックサム1バイト）
  const u16 totalPacketSize = 7 + (dataPerServo + 1) * IDN + 1;
  if (IDN == 0 || totalPacketSize > MAX_PACKET_SIZE)
  {
    return 0;
  }
  const u8 messageLength = ((dataPerServo + 1) * IDN + 4); // +1はID, +4は基本ヘッダ

  u16 packetIndex = 0;

  // ヘッダ構築
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xfe; // ブロードキャストID
  packet[packetIndex++] = messageLength;
  packet[packetIndex++] = INST_SYNC_WRITE;
  packet[packetIndex++] = startAddress; // 開始レジスタアドレス
  packet[packetIndex++] = dataPerServo; // 各サーボのデータ長

  // 各サーボのIDとデータをパケットに追加
  for (u8 i = 0; i < IDN; i++)
  {
    s16 pos = Position[i];

    // 負の位置の処理（STS仕様の方向ビット）
    if (!isSCS && pos < 0)
    {
      pos = -pos;
      pos |= (1 << 15);
    }

    packet[packetIndex++] = ID[i];
    if (!isSCS)
    {
      packet[packetIndex++] = ACC ? ACC[i] : 0; // 加速度
    }
    encodeU16(pos, &packet[packetIndex]); // 位置
    packetIndex += 2;
    encodeU16(0, &packet[packetIndex]); // 時間（0固定）
    packetIndex += 2;
    encodeU16(Speed ? Speed[i] : 0, &packet[packetIndex]); // 速度
    packetIndex += 2;
  }

  // チェックサムを追加
  packet[packetIndex] = calcChecksum(packet, packetIndex + 1);
  packetIndex++;

  return packetIndex;
}

// SyncWrite 1パケット送信（一括送信最適化版、STS/SCS対応）
bool syncWritePosSingle(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
  u8 packet[MAX_PACKET_SIZE];
  u16 packetSize = buildSyncWritePosPacket(packet, ID, IDN, Position, Speed, ACC);
  if (packetSize == 0)
  {
    Serial.println("Error: Invalid servo count or packet size too large");
    return false;
  }

  Serial.print("SyncWrite (");
  Serial.print(servoTypeName(servoType));
  Serial.print("): ");
  Serial.print(IDN);
  Serial.print(" servos, packet size: ");
  Serial.print(packetSize);
  Serial.println(" bytes");

  for (u8 i = 0; i < IDN; i++)
  {
    Serial.print("  ID ");
    Serial.print(ID[i]);
    Serial.print(": Pos=");
    Serial.print(positionToAngle(Position[i]), 1);
    Serial.print("deg (");
    Serial.print(Position[i]);
    Serial.print("), Speed=");
    Serial.print(Speed ? Speed[i] : 0);
    if (servoType != SERVO_TYPE_SCS)
    {
      Serial.print(", ACC=");
      Serial.print(ACC ? ACC[i] : 0);
    }
    Serial.println();
  }

  // 半二重制御での送信
  flushReceiveBuffer();
  busDirTx();
  delayMicroseconds(30);

  servoSerial->write(packet, packetSize);
  servoSerial->flush();

  // 送信専用モードを維持（SyncWriteは応答なし）
  delayMicroseconds(50);

  Serial.println("SyncWrite packet sent successfully");
  return true;
}

// SyncWritePos実装（1パケットの上限台数を超える場合は分割送信）
bool SyncWritePos(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
  if (IDN == 0)
  {
    Serial.println("Error: Invalid servo count");
    return false;
  }

  // 1パケットの上限台数（STS: 31台, SCS: 35台）
  const u8 dataPerServo = (servoType == SERVO_TYPE_SCS) ? 6 : 7;
  const u8 maxPerPacket = (MAX_PACKET_SIZE - 8) / (dataPerServo + 1);

  bool ok = true;
  for (u16 offset = 0; offset < IDN; offset += maxPerPacket)
  {
    u8 n = (IDN - offset < maxPerPacket) ? (IDN - offset) : maxPerPacket;
    ok &= syncWritePosSingle(&ID[offset], n, &Position[offset],
                             Speed ? &Speed[offset] : NULL, ACC ? &ACC[offset] : NULL);
  }
  return ok;
}

// トルクイネーブル（半二重通信対応版）
bool EnableTorque(u8 ID, u8 Enable)
{
  // パケットバッファ準備
  u8 packet[8]; // 最大8バイト
  u8 packetIndex = 0;
  u8 messageLength = 4;

  // ヘッダ構築
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = ID;
  packet[packetIndex++] = messageLength;
  packet[packetIndex++] = CMD_WRITE;             // 通常のWRITE命令を使用
  packet[packetIndex++] = SMS_STS_TORQUE_ENABLE; // レジスタアドレス40

  // データ追加
  packet[packetIndex++] = Enable; // 1=トルクオン, 0=トルクオフ

  // チェックサム計算と追加
  u8 checksum = ID + messageLength + CMD_WRITE + SMS_STS_TORQUE_ENABLE + Enable;
  packet[packetIndex++] = ~checksum;

  // 半二重制御での送信
  flushReceiveBuffer();
  busDirTx();
  delayMicroseconds(30);

  // パケット送信
  servoSerial->write(packet, packetIndex);
  servoSerial->flush();

  // 送信専用モードを維持
  delayMicroseconds(50);

  return true;
}

// 汎用パケット構築・送信関数
bool sendPacket(u8 targetID, u8 instruction, u8 startAddress, u8 *data, u8 dataLength)
{
  // パケットサイズ計算
  u8 messageLength = dataLength ? (dataLength + 3) : 2;
  u8 totalSize = dataLength ? (6 + dataLength + 1) : (5 + 1);

  if (totalSize > MAX_PACKET_SIZE)
  {
    Serial.println("Error: Packet too large");
    return false;
  }

  // パケットバッファ準備
  u8 packet[MAX_PACKET_SIZE];
  u8 packetIndex = 0;

  // ヘッダ構築
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = targetID;
  packet[packetIndex++] = messageLength;
  packet[packetIndex++] = instruction;

  // アドレスとデータ（存在する場合）
  u8 checksum = targetID + messageLength + instruction;
  if (data && dataLength > 0)
  {
    packet[packetIndex++] = startAddress;
    checksum += startAddress;

    for (u8 i = 0; i < dataLength; i++)
    {
      packet[packetIndex++] = data[i];
      checksum += data[i];
    }
  }

  // チェックサム追加
  packet[packetIndex++] = ~checksum;

  // 一括送信
  writeServo(packet, packetIndex);
  flushServo();

  return true;
}

// ★ ヘルパー関数: 角度から位置値への変換
s16 angleToPosition(float angle)
{
  // 角度範囲チェック（STS: ±180°, SCS: ±150°）
  float halfRange = angleRange() / 2.0;
  if (angle < -halfRange || angle > halfRange)
  {
    return -1; // エラー値
  }

  // 角度を位置値に変換（0° = 中央位置, STS: 0-4095, SCS: 0-1023）
  s16 position = (s16)(positionCenter() + (angle / angleRange()) * (positionMax() + 1));
  if (position < POSITION_MIN)
    position = POSITION_MIN;
  if (position > (s16)positionMax())
    position = positionMax();

  return position;
}

// // 設定関数
// void setServoSerial(HardwareSerial *serial)
// {
//   servoSerial = serial;
// }

// void setIOTimeout(unsigned long timeout)
// {
//   ioTimeout = timeout;
// }

// void setResponseLevel(u8 level)
// {
//   responseLevel = level;
// }

// サーボ設定
// u8 armIDs[] = {1, 2, 3, 4, 5, 6};
// s16 ServoPositions[] = {0, 0, 0, 0, 0, 0};
// u16 ServoSpeeds[] = {0, 0, 0, 0, 0, 0};
// u8 ServoACCs[] = {0, 0, 0, 0, 0, 0};

// 往復移動用変数
bool currentDirection = true; // true: 4000へ, false: 500へ
unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 2000; // 2秒間隔

// 往復移動の実行（最適化版）
void executeReciprocationSync()
{
  unsigned long currentTime = millis();

  if (currentTime - lastMoveTime >= MOVE_INTERVAL)
  {
    // 目標位置を決定
    s16 targetPosition = currentDirection ? 4000 : 500;

    Serial.print("Moving to ");
    Serial.print(currentDirection ? "MAX" : "MIN");
    Serial.print(" position: ");
    Serial.println(targetPosition);

    // アクティブなサーボ（ID1,2）の設定
    u8 activeIDs[] = {1, 2};
    s16 activePositions[] = {targetPosition, targetPosition};
    u16 activeSpeeds[] = {1000, 1000};
    u8 activeACCs[] = {50, 50};

    // SyncWritePosで一括送信（最適化版）
    SyncWritePos(activeIDs, 2, activePositions, activeSpeeds, activeACCs);

    // 方向切り替え
    currentDirection = !currentDirection;
    lastMoveTime = currentTime;

    Serial.print("Next move in ");
    Serial.print(MOVE_INTERVAL / 1000);
    Serial.println(" seconds");
    Serial.println();
  }
}

// [r]コマンド: サーボ位置読み取り
void executeReadPositionCommand()
{
  Serial.print("Reading position from servo ID ");
  Serial.println(targetServoID);

  // 読み取りコマンド構築
  u8 command[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, REG_CURRENT_POSITION, 2, 0};
  command[7] = calcChecksum(command, 8);

  // コマンド送信
  sendCommand(command, 8);

  // 応答受信（8バイト期待）
  u8 response[8];
  if (receiveResponse(response, 8, 100))
  {
    // 応答検証
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      u16 position = decodeU16(&response[5], servoType);
      float angle = positionToAngle(position);

      Serial.print("Position: ");
      Serial.print(position);
      Serial.print(" (");
      Serial.print(angle, 1);
      Serial.println("°)");
    }
    else
    {
      Serial.println("Invalid response received");
    }
  }
  else
  {
    Serial.println("No response from servo");
  }

  // 送信専用モードに確実に復帰
  busDirTx();
}

// [v]コマンド: サーボ電圧読み取り
void executeReadVoltageCommand()
{
  Serial.print("Reading voltage from servo ID ");
  Serial.println(targetServoID);

  // 読み取りコマンド構築
  u8 command[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, REG_VOLTAGE, 1, 0};
  command[7] = calcChecksum(command, 8);

  // コマンド送信
  sendCommand(command, 8);

  // 応答受信（7バイト期待）
  u8 response[7];
  if (receiveResponse(response, 7, 100))
  {
    // 応答検証
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      u8 voltage = response[5];
      float voltageV = voltage / 10.0;

      Serial.print("Voltage: ");
      Serial.print(voltage);
      Serial.print(" (");
      Serial.print(voltageV, 1);
      Serial.print("V)");

      if (voltage < VOLTAGE_MIN)
      {
        Serial.println(" [Warning] Low voltage");
      }
      else if (voltage > VOLTAGE_MAX)
      {
        Serial.println(" [Warning] High voltage");
      }
      else
      {
        Serial.println(" [OK]");
      }
    }
    else
    {
      Serial.println("Invalid response received");
    }
  }
  else
  {
    Serial.println("No response from servo");
  }

  // 送信専用モードに確実に復帰
  busDirTx();
}

// [z]コマンド: ターゲットID変更
void executeChangeTargetIdCommand()
{
  Serial.print("Current target ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter new target ID (1-253): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  int newId = input.toInt();

  // ID範囲チェック
  if (newId < 1 || newId > 253)
  {
    Serial.println("Error: ID out of range (1-253)");
    return;
  }

  targetServoID = newId;
  Serial.print("Target ID changed to: ");
  Serial.println(targetServoID);
}

// 一斉制御IDリストを表示（例: 1,3,7）
void printSyncServoIDs()
{
  for (u8 i = 0; i < syncServoCount; i++)
  {
    Serial.print(syncServoIDs[i]);
    if (i < syncServoCount - 1)
      Serial.print(",");
  }
}

// IDリスト文字列を解析（例: "1,3,7" / "1-4,7"）。昇順・重複なしでoutに格納し、個数を返す（-1はエラー）
int parseIdList(String input, u8 *out)
{
  bool selected[254] = {false};
  input += ","; // 最後のトークンも区切りで処理

  int tokenStart = 0;
  for (int pos = 0; pos < (int)input.length(); pos++)
  {
    if (input[pos] != ',')
      continue;

    String token = input.substring(tokenStart, pos);
    token.trim();
    tokenStart = pos + 1;
    if (token.length() == 0)
      continue;

    // 数字と'-'（1個まで）以外は不正
    int dashCount = 0;
    for (unsigned int k = 0; k < token.length(); k++)
    {
      if (token[k] == '-')
        dashCount++;
      else if (!isDigit(token[k]))
        return -1;
    }
    if (dashCount > 1)
      return -1;

    // 範囲指定（a-b）または単一ID
    int dash = token.indexOf('-');
    int first = token.toInt();
    int last = dash >= 0 ? token.substring(dash + 1).toInt() : first;
    if (first < 1 || last > 253 || first > last)
      return -1;

    for (int id = first; id <= last; id++)
      selected[id] = true;
  }

  int count = 0;
  for (int id = 1; id <= 253; id++)
  {
    if (selected[id])
      out[count++] = id;
  }
  return count > 0 ? count : -1;
}

// [n]コマンド: 一斉制御サーボIDリスト設定
void executeSetSyncServoIDsCommand()
{
  Serial.print("Current sync servo IDs: ");
  printSyncServoIDs();
  Serial.println();
  Serial.println("Enter sync servo IDs (e.g. 1,3,7 or 1-4,7): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  u8 newIDs[MAX_SYNC_SERVOS];
  int newCount = parseIdList(input, newIDs);
  if (newCount < 0)
  {
    Serial.println("Error: Invalid ID list (valid IDs: 1-253)");
    return;
  }

  memcpy(syncServoIDs, newIDs, newCount);
  syncServoCount = newCount;
  Serial.print("Sync servo IDs changed to: ");
  printSyncServoIDs();
  Serial.print(" (");
  Serial.print(syncServoCount);
  Serial.println(" servos)");
}

// [a]コマンド: 全サーボ位置読み出し
void executeReadAllPositionsCommand()
{
  Serial.print("Reading positions from ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");
  Serial.println("ID\tPosition\tAngle(°)");
  Serial.println("-------------------------");

  u8 connectedCount = 0;

  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    // 位置読み取りコマンド構築
    u8 command[8] = {0xFF, 0xFF, id, 4, CMD_READ, REG_CURRENT_POSITION, 2, 0};
    command[7] = calcChecksum(command, 8);

    // コマンド送信
    sendCommand(command, 8);

    // 応答受信（8バイト期待）
    u8 response[8];
    if (receiveResponse(response, 8, 100))
    {
      // 応答検証
      if (response[0] == 0xFF && response[1] == 0xFF && response[2] == id)
      {
        u16 position = decodeU16(&response[5], servoType);
        float angle = positionToAngle(position);

        Serial.print(id);
        Serial.print("\t");
        Serial.print(position);
        Serial.print("\t\t");
        Serial.print(angle, 1);
        Serial.println("°");

        connectedCount++;
      }
      else
      {
        Serial.print(id);
        Serial.println("\t[Error] Invalid response");
      }
    }
    else
    {
      Serial.print(id);
      Serial.println("\t[Error] No response");
    }

    delay(10); // 次の読み取りまで少し待機
  }

  Serial.println("-------------------------");
  Serial.print("Connected servos: ");
  Serial.print(connectedCount);
  Serial.print("/");
  Serial.println(syncServoCount);

  // 送信専用モードに確実に復帰
  busDirTx();
}

// [c]コマンド: 全サーボ位置を0にリセット
void executeResetAllPositionsCommand()
{
  Serial.print("Resetting ");
  Serial.print(syncServoCount);
  Serial.println(" servos to position 0 (center)...");

  // 動的配列で一斉制御サーボ数に対応
  s16 *allPositions = new s16[syncServoCount];
  u16 *allSpeeds = new u16[syncServoCount];
  u8 *allACCs = new u8[syncServoCount];

  // 配列を初期化（全て位置0 = 中央位置）
  for (u8 i = 0; i < syncServoCount; i++)
  {
    allPositions[i] = positionCenter(); // 中央位置（0度）
    allSpeeds[i] = 800;     // 少し遅めの速度
    allACCs[i] = 30;        // 低い加速度
  }

  SyncWritePos(syncServoIDs, syncServoCount, allPositions, allSpeeds, allACCs);

  Serial.println("All servos reset to center position");

  // メモリ解放
  delete[] allPositions;
  delete[] allSpeeds;
  delete[] allACCs;
}

// PINGで指定IDのサーボ応答を確認
bool pingServo(u8 id)
{
  u8 command[6] = {0xFF, 0xFF, id, 2, CMD_PING, 0};
  command[5] = calcChecksum(command, 6);
  sendCommand(command, 6);

  u8 response[6];
  bool found = receiveResponse(response, 6, 30) &&
               response[0] == 0xFF && response[1] == 0xFF && response[2] == id;

  // 送信専用モードに確実に復帰
  busDirTx();
  return found;
}

// 1バイトレジスタ読み取り（成功時true）
bool readByteRegister(u8 id, u8 address, u8 *value, unsigned long timeout)
{
  u8 command[8] = {0xFF, 0xFF, id, 4, CMD_READ, address, 1, 0};
  command[7] = calcChecksum(command, 8);
  sendCommand(command, 8);

  u8 response[7];
  bool ok = receiveResponse(response, 7, timeout) &&
            response[0] == 0xFF && response[1] == 0xFF && response[2] == id;
  if (ok)
  {
    *value = response[5];
  }

  // 送信専用モードに確実に復帰
  busDirTx();
  return ok;
}

// [f]コマンド: サーボ認識（STS/SCS系列の自動判定付き）
void executeServoDetectionCommand()
{
  Serial.print("Enter maximum ID to scan (1-253, empty = ");
  Serial.print(DEFAULT_SCAN_MAX_ID);
  Serial.println("): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  // 空欄リターンはデフォルト最大IDで検索
  int maxId = input.length() == 0 ? DEFAULT_SCAN_MAX_ID : input.toInt();

  // ID範囲チェック
  if (maxId < 1 || maxId > 253)
  {
    Serial.println("Error: ID out of range (1-253)");
    return;
  }

  Serial.print("Scanning servos from ID1 to ID");
  Serial.print(maxId);
  Serial.println("...");
  Serial.println();

  u8 foundServos[254];     // 最大253個のサーボ
  u8 foundTypes[254];      // 各サーボの系列（0xFF = 判定不能）
  u8 foundCount = 0;
  u8 typeCounts[2] = {0, 0}; // [STS, SCS]の検出数

  // プログレス表示用
  int progressStep = maxId / 10;
  if (progressStep == 0)
    progressStep = 1;

  for (u8 id = 1; id <= maxId; id++)
  {
    // プログレス表示
    if (id % progressStep == 0 || id == maxId)
    {
      Serial.print("Progress: ");
      Serial.print((id * 100) / maxId);
      Serial.println("%");
    }

    if (pingServo(id))
    {
      // アドレス2(END)で系列判定: 0=STS, 1=SCS
      u8 endValue;
      u8 type = 0xFF;
      if (readByteRegister(id, REG_END, &endValue, 30) && endValue <= SERVO_TYPE_SCS)
      {
        type = endValue;
        typeCounts[type]++;
      }

      foundServos[foundCount] = id;
      foundTypes[foundCount] = type;
      foundCount++;
      Serial.print("Found servo ID: ");
      Serial.print(id);
      Serial.print(" (");
      Serial.print(type == 0xFF ? "Unknown" : servoTypeName(type));
      Serial.println(")");
    }

    delay(5); // 次のスキャンまで少し待機
  }

  // 結果表示
  Serial.println();
  Serial.println("=== Servo Detection Results ===");
  Serial.print("Scanned range: ID1 to ID");
  Serial.println(maxId);
  Serial.print("Found ");
  Serial.print(foundCount);
  Serial.print(" servo(s): STS=");
  Serial.print(typeCounts[SERVO_TYPE_STS]);
  Serial.print(", SCS=");
  Serial.println(typeCounts[SERVO_TYPE_SCS]);

  if (foundCount > 0)
  {
    Serial.print("Connected IDs: ");
    for (u8 i = 0; i < foundCount; i++)
    {
      Serial.print(foundServos[i]);
      Serial.print("(");
      Serial.print(foundTypes[i] == 0xFF ? "?" : servoTypeName(foundTypes[i]));
      Serial.print(")");
      if (i < foundCount - 1)
        Serial.print(", ");
    }
    Serial.println();

    // 検出結果からサーボ系列を自動設定
    if (typeCounts[SERVO_TYPE_STS] + typeCounts[SERVO_TYPE_SCS] > 0)
    {
      servoType = (typeCounts[SERVO_TYPE_SCS] > typeCounts[SERVO_TYPE_STS]) ? SERVO_TYPE_SCS : SERVO_TYPE_STS;
      Serial.print("Servo series auto-set to: ");
      Serial.println(servoTypeName(servoType));

      if (typeCounts[SERVO_TYPE_STS] > 0 && typeCounts[SERVO_TYPE_SCS] > 0)
      {
        Serial.println("[Warning] STS and SCS are mixed on the bus.");
        Serial.print("          Only ");
        Serial.print(servoTypeName(servoType));
        Serial.println(" servos are added to sync servo IDs.");
      }
    }

    // 見つかったIDにターゲットを自動変更（設定系列に一致する最小IDを優先）
    targetServoID = foundServos[0];
    for (u8 i = 0; i < foundCount; i++)
    {
      if (foundTypes[i] == servoType)
      {
        targetServoID = foundServos[i];
        break;
      }
    }
    Serial.print("Target ID auto-set to: ");
    Serial.println(targetServoID);

    // 見つかったIDを一斉制御リストに自動設定（設定系列に一致するもの。なければ全て）
    u8 count = 0;
    for (u8 i = 0; i < foundCount; i++)
    {
      if (foundTypes[i] == servoType)
        syncServoIDs[count++] = foundServos[i];
    }
    if (count == 0)
    {
      memcpy(syncServoIDs, foundServos, foundCount);
      count = foundCount;
    }
    syncServoCount = count;
    Serial.print("Sync servo IDs auto-set to: ");
    printSyncServoIDs();
    Serial.println();
  }
  else
  {
    Serial.println("No servos detected");
  }

  Serial.println();
}

// [e]コマンド: 全サーボトルクオン
void executeEnableTorqueAllCommand()
{
  Serial.print("Enabling torque for ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");

  u8 successCount = 0;

  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    if (EnableTorque(id, 1))
    {
      Serial.print("ID ");
      Serial.print(id);
      Serial.println(": Torque enabled");
      successCount++;
    }
    else
    {
      Serial.print("ID ");
      Serial.print(id);
      Serial.println(": Failed to enable torque");
    }
    delay(50); // 各サーボ間で少し待機
  }

  Serial.print("Torque enabled for ");
  Serial.print(successCount);
  Serial.print("/");
  Serial.print(syncServoCount);
  Serial.println(" servos");
}

// [d]コマンド: 全サーボトルクオフ
void executeDisableTorqueAllCommand()
{
  Serial.print("Disabling torque for ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");

  u8 successCount = 0;

  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    if (EnableTorque(id, 0))
    {
      Serial.print("ID ");
      Serial.print(id);
      Serial.println(": Torque disabled");
      successCount++;
    }
    else
    {
      Serial.print("ID ");
      Serial.print(id);
      Serial.println(": Failed to disable torque");
    }
    delay(50); // 各サーボ間で少し待機
  }

  Serial.print("Torque disabled for ");
  Serial.print(successCount);
  Serial.print("/");
  Serial.print(syncServoCount);
  Serial.println(" servos");
}

// [p]コマンド: 指定アドレスのレジスタ値読み取り
void executeReadRegisterCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter register address (0-255): ");

  // アドレス入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String addressInput = Serial.readString();
  addressInput.trim();

  int address = addressInput.toInt();

  // アドレス範囲チェック
  if (address < 0 || address > 255)
  {
    Serial.println("Error: Address out of range (0-255)");
    return;
  }

  Serial.println("Enter data length to read (1-4 bytes): ");

  // データ長入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String lengthInput = Serial.readString();
  lengthInput.trim();

  int dataLength = lengthInput.toInt();

  // データ長範囲チェック
  if (dataLength < 1 || dataLength > 4)
  {
    Serial.println("Error: Data length out of range (1-4)");
    return;
  }

  Serial.print("Reading ");
  Serial.print(dataLength);
  Serial.print(" byte(s) from address ");
  Serial.print(address);
  Serial.print(" of servo ID ");
  Serial.println(targetServoID);

  // 読み取りコマンド構築
  u8 command[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, (u8)address, (u8)dataLength, 0};
  command[7] = calcChecksum(command, 8);

  // コマンド送信
  sendCommand(command, 8);

  // 応答受信（ヘッダ5バイト + データ + チェックサム1バイト）
  u8 expectedLength = 5 + dataLength + 1;
  u8 response[10]; // 最大10バイト

  if (receiveResponse(response, expectedLength, 100))
  {
    // 応答検証
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      Serial.print("Register ");
      Serial.print(address);
      Serial.print(" (0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.print("): ");

      // データ部分を表示
      if (dataLength == 1)
      {
        u8 value = response[5];
        Serial.print("Dec=");
        Serial.print(value);
        Serial.print(", Hex=0x");
        if (value < 16)
          Serial.print("0");
        Serial.print(value, HEX);
        Serial.print(", Bin=");
        Serial.println(value, BIN);
      }
      else if (dataLength == 2)
      {
        u16 value = decodeU16(&response[5], servoType); // 系列別エンディアン
        Serial.print("Dec=");
        Serial.print(value);
        Serial.print(", Hex=0x");
        if (value < 0x1000)
          Serial.print("0");
        if (value < 0x100)
          Serial.print("0");
        if (value < 0x10)
          Serial.print("0");
        Serial.print(value, HEX);
        Serial.print(", Bin=");
        Serial.println(value, BIN);
      }
      else
      {
        // 3-4バイトの場合は個別に表示
        Serial.print("Raw bytes: ");
        for (int i = 0; i < dataLength; i++)
        {
          u8 value = response[5 + i];
          Serial.print("0x");
          if (value < 16)
            Serial.print("0");
          Serial.print(value, HEX);
          if (i < dataLength - 1)
            Serial.print(" ");
        }
        Serial.println();
      }
    }
    else
    {
      Serial.println("Invalid response received");
      Serial.print("Response header: ");
      for (int i = 0; i < 3; i++)
      {
        Serial.print("0x");
        if (response[i] < 16)
          Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  else
  {
    Serial.println("No response from servo");
  }

  // 送信専用モードに確実に復帰
  busDirTx();
}

// [s]コマンド: 指定アドレスにレジスタ値書き込み
void executeWriteRegisterCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter register address (0-255): ");

  // アドレス入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String addressInput = Serial.readString();
  addressInput.trim();

  int address = addressInput.toInt();

  // アドレス範囲チェック
  if (address < 0 || address > 255)
  {
    Serial.println("Error: Address out of range (0-255)");
    return;
  }

  Serial.println("Enter data length to write (1-2 bytes): ");

  // データ長入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String lengthInput = Serial.readString();
  lengthInput.trim();

  int dataLength = lengthInput.toInt();

  // データ長範囲チェック
  if (dataLength < 1 || dataLength > 2)
  {
    Serial.println("Error: Data length out of range (1-2)");
    return;
  }

  Serial.print("Enter decimal value to write (");
  if (dataLength == 1)
  {
    Serial.print("0-255");
  }
  else
  {
    Serial.print("0-65535");
  }
  Serial.println("): ");

  // 値入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String valueInput = Serial.readString();
  valueInput.trim();

  long value = valueInput.toInt();

  // 値範囲チェック
  if (dataLength == 1 && (value < 0 || value > 255))
  {
    Serial.println("Error: Value out of range (0-255) for 1 byte");
    return;
  }
  else if (dataLength == 2 && (value < 0 || value > 65535))
  {
    Serial.println("Error: Value out of range (0-65535) for 2 bytes");
    return;
  }

  Serial.print("Writing ");
  Serial.print(value);
  Serial.print(" (0x");
  if (value < 16)
    Serial.print("0");
  Serial.print(value, HEX);
  Serial.print(") to address ");
  Serial.print(address);
  Serial.print(" of servo ID ");
  Serial.println(targetServoID);

  // 書き込みコマンド構築
  u8 command[10]; // 最大10バイト
  u8 commandIndex = 0;
  u8 messageLength = 3 + dataLength;

  // ヘッダ構築
  command[commandIndex++] = 0xFF;
  command[commandIndex++] = 0xFF;
  command[commandIndex++] = targetServoID;
  command[commandIndex++] = messageLength;
  command[commandIndex++] = CMD_WRITE;
  command[commandIndex++] = (u8)address;

  // データ追加
  if (dataLength == 1)
  {
    command[commandIndex++] = (u8)value;
  }
  else // dataLength == 2
  {
    // 系列別エンディアンで送信（STS: リトル, SCS: ビッグ）
    encodeU16((u16)value, &command[commandIndex]);
    commandIndex += 2;
  }

  // チェックサム計算
  command[commandIndex] = calcChecksum(command, commandIndex + 1);
  commandIndex++;

  // 半二重制御での送信
  flushReceiveBuffer();
  busDirTx();
  delayMicroseconds(30);

  // コマンド送信
  servoSerial->write(command, commandIndex);
  servoSerial->flush();

  // 送信専用モードを維持
  delayMicroseconds(50);

  Serial.println("Write command sent successfully");

  // 確認のため値を読み返す
  Serial.println("Verifying written value...");
  delay(50);

  // 読み取りコマンド構築
  u8 readCommand[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, (u8)address, (u8)dataLength, 0};
  readCommand[7] = calcChecksum(readCommand, 8);

  // 読み取りコマンド送信
  sendCommand(readCommand, 8);

  // 応答受信8
  u8 expectedLength = 5 + dataLength + 1;
  u8 response[10];

  if (receiveResponse(response, expectedLength, 100))
  {
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      if (dataLength == 1)
      {
        u8 readValue = response[5];
        Serial.print("Verification: Read value = ");
        Serial.print(readValue);
        Serial.print(" (0x");
        if (readValue < 16)
          Serial.print("0");
        Serial.print(readValue, HEX);
        Serial.print(")");
        if (readValue == (u8)value)
        {
          Serial.println(" [OK]");
        }
        else
        {
          Serial.println(" [MISMATCH]");
        }
      }
      else
      {
        u16 readValue = decodeU16(&response[5], servoType);
        Serial.print("Verification: Read value = ");
        Serial.print(readValue);
        Serial.print(" (0x");
        if (readValue < 0x1000)
          Serial.print("0");
        if (readValue < 0x100)
          Serial.print("0");
        if (readValue < 0x10)
          Serial.print("0");
        Serial.print(readValue, HEX);
        Serial.print(")");
        if (readValue == (u16)value)
        {
          Serial.println(" [OK]");
        }
        else
        {
          Serial.println(" [MISMATCH]");
        }
      }
    }
    else
    {
      Serial.println("Verification failed: Invalid response");
    }
  }
  else
  {
    Serial.println("Verification failed: No response");
  }

  // 送信専用モードに確実に復帰
  busDirTx();
}

// [x]コマンド: 全サーボに角度を一斉送信v
void executeAngleCommand()
{
  float halfRange = angleRange() / 2.0;
  Serial.print("Enter angle (");
  Serial.print(-halfRange, 0);
  Serial.print(" to ");
  Serial.print(halfRange, 0);
  Serial.println(" degrees): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  float angle = input.toFloat();

  // 角度範囲チェック
  if (angle < -halfRange || angle > halfRange)
  {
    Serial.println("Error: Angle out of range");
    return;
  }

  // 角度を位置値に変換
  s16 position = angleToPosition(angle);
  if (position == -1)
  {
    Serial.println("Error: Invalid angle");
    return;
  }

  Serial.print("Converting ");
  Serial.print(angle);
  Serial.print(" degrees to position ");
  Serial.println(position);

  // 動的配列で一斉制御サーボ数に対応
  s16 *allPositions = new s16[syncServoCount];
  u16 *allSpeeds = new u16[syncServoCount];
  u8 *allACCs = new u8[syncServoCount];

  // 配列を初期化
  for (u8 i = 0; i < syncServoCount; i++)
  {
    allPositions[i] = position;
    allSpeeds[i] = 1000;
    allACCs[i] = 50;
  }

  Serial.print("Sending angle command to ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");

  SyncWritePos(syncServoIDs, syncServoCount, allPositions, allSpeeds, allACCs);

  Serial.println("Angle command sent successfully");

  // メモリ解放
  delete[] allPositions;
  delete[] allSpeeds;
  delete[] allACCs;
}

// 1バイトレジスタ書き込み（応答なし、半二重通信対応）
void writeByteRegister(u8 ID, u8 address, u8 value)
{
  u8 packet[8];
  u8 messageLength = 4;

  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = ID;
  packet[3] = messageLength;
  packet[4] = CMD_WRITE;
  packet[5] = address;
  packet[6] = value;
  packet[7] = ~(u8)(ID + messageLength + CMD_WRITE + address + value);

  // 半二重制御での送信
  flushReceiveBuffer();
  busDirTx();
  delayMicroseconds(30);

  servoSerial->write(packet, 8);
  servoSerial->flush();

  // 送信専用モードを維持
  delayMicroseconds(50);
}

// ESP32側のサーボ通信速度を変更
void setHostBaudRate(u8 baudIndex)
{
  servoSerial->flush();
  servoSerial->updateBaudRate(STS_BAUD_RATES[baudIndex]);
  servoBaudIndex = baudIndex;
  delay(10);
  flushReceiveBuffer();
}

// [b]コマンド: 通信速度設定
void executeSetBaudRateCommand()
{
  Serial.print("Current ESP32 baud rate: ");
  Serial.print(STS_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  Serial.println("STS standard baud rates:");
  for (u8 i = 0; i < STS_BAUD_COUNT; i++)
  {
    Serial.print("  [");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(STS_BAUD_RATES[i]);
    Serial.println(" bps");
  }
  Serial.print("Enter baud rate number (0-");
  Serial.print(STS_BAUD_COUNT - 1);
  Serial.println("): ");

  // 番号入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String baudInput = Serial.readString();
  baudInput.trim();

  // 範囲チェック（toInt()は非数値で0を返すため数字であることも確認）
  int baudIndex = baudInput.toInt();
  if (baudInput.length() != 1 || !isDigit(baudInput[0]) || baudIndex < 0 || baudIndex >= (int)STS_BAUD_COUNT)
  {
    Serial.println("Error: Baud rate number out of range");
    return;
  }

  Serial.println("Apply to: [1] ESP32 only");
  Serial.print("          [2] Servo ID ");
  Serial.print(targetServoID);
  Serial.println(" (EEPROM) + ESP32");

  // 適用先入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String modeInput = Serial.readString();
  modeInput.trim();

  if (modeInput == "1")
  {
    setHostBaudRate(baudIndex);
    Serial.print("ESP32 baud rate changed to: ");
    Serial.print(STS_BAUD_RATES[servoBaudIndex]);
    Serial.println(" bps");
    return;
  }
  else if (modeInput != "2")
  {
    Serial.println("Error: Invalid selection");
    return;
  }

  Serial.print("Writing baud rate ");
  Serial.print(STS_BAUD_RATES[baudIndex]);
  Serial.print(" bps to servo ID ");
  Serial.println(targetServoID);

  // EEPROMロック解除 -> ボーレート書き込み（現在の速度で送信）
  writeByteRegister(targetServoID, regLock(), 0);
  delay(10);
  writeByteRegister(targetServoID, REG_BAUD_RATE, (u8)baudIndex);
  delay(50);

  // サーボは書き込み直後に新速度へ切り替わるため、ESP32側も追従
  setHostBaudRate(baudIndex);

  // EEPROMロック（新しい速度で送信）
  writeByteRegister(targetServoID, regLock(), 1);
  delay(50);

  // 新しい速度で読み返して確認
  Serial.println("Verifying at new baud rate...");
  u8 command[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, REG_BAUD_RATE, 1, 0};
  command[7] = calcChecksum(command, 8);
  sendCommand(command, 8);

  u8 response[7];
  if (receiveResponse(response, 7, 100) &&
      response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
  {
    Serial.print("Verification: Baud rate register = ");
    Serial.print(response[5]);
    if (response[5] == baudIndex)
    {
      Serial.println(" [OK]");
    }
    else
    {
      Serial.println(" [MISMATCH]");
    }
  }
  else
  {
    Serial.println("Verification failed: No response at new baud rate");
    Serial.println("Hint: Use [b] -> [1] to try other ESP32 baud rates");
  }

  Serial.print("ESP32 baud rate: ");
  Serial.print(STS_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");

  // 送信専用モードに確実に復帰
  busDirTx();
}

// [i]コマンド: サーボID変更
void executeChangeServoIdCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);

  // 変更元サーボの存在確認
  if (!pingServo(targetServoID))
  {
    Serial.println("Error: No response from target servo (use [z] to select ID)");
    return;
  }

  Serial.println("Enter new servo ID (1-253): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  int newId = input.toInt();

  // ID範囲チェック
  if (newId < 1 || newId > 253)
  {
    Serial.println("Error: ID out of range (1-253)");
    return;
  }

  if (newId == targetServoID)
  {
    Serial.println("New ID is the same as current ID. Nothing to do.");
    return;
  }

  // ID重複チェック
  if (pingServo((u8)newId))
  {
    Serial.print("Error: ID ");
    Serial.print(newId);
    Serial.println(" is already in use");
    return;
  }

  Serial.print("Changing servo ID ");
  Serial.print(targetServoID);
  Serial.print(" -> ");
  Serial.println(newId);

  // EEPROMロック解除 -> ID書き込み（旧IDで送信）
  writeByteRegister(targetServoID, regLock(), 0);
  delay(10);
  writeByteRegister(targetServoID, REG_ID, (u8)newId);
  delay(50);

  // EEPROMロック（書き込み直後に新IDへ切り替わるため新IDで送信）
  writeByteRegister((u8)newId, regLock(), 1);
  delay(50);

  // 新IDで応答確認
  if (pingServo((u8)newId))
  {
    targetServoID = newId;
    Serial.print("Servo ID changed successfully [OK]. Target ID is now ");
    Serial.println(targetServoID);
  }
  else
  {
    Serial.println("Verification failed: No response at new ID");
    Serial.println("Hint: Use [f] to scan servos");
  }
}

// [m]コマンド: サーボのバージョン情報（ファームウェア/型番）読み取り
void executeReadVersionCommand()
{
  Serial.print("Reading version info from servo ID ");
  Serial.println(targetServoID);

  // アドレス0から5バイト読み取り（FW主/副, END, サーボ主/副）
  u8 command[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, REG_FIRMWARE_MAJOR, 5, 0};
  command[7] = calcChecksum(command, 8);
  sendCommand(command, 8);

  // 応答受信（ヘッダ5バイト + データ5バイト + チェックサム1バイト）
  u8 response[11];
  if (receiveResponse(response, 11, 100))
  {
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      u8 fwMajor = response[5];
      u8 fwMinor = response[6];
      u8 servoMajor = response[8];
      u8 servoMinor = response[9];
      u8 endValue = response[7];
      u8 type = endValue == SERVO_TYPE_SCS ? SERVO_TYPE_SCS : SERVO_TYPE_STS;
      u16 modelNumber = decodeU16(&response[8], type); // アドレス3-4（サーボ自身のエンディアン）

      // 既知の型番を検索
      const char *modelName = "Unknown";
      for (u8 i = 0; i < STS_MODEL_COUNT; i++)
      {
        if (STS_MODELS[i].number == modelNumber)
        {
          modelName = STS_MODELS[i].name;
          break;
        }
      }

      Serial.print("Servo series    : ");
      Serial.print(servoTypeName(type));
      Serial.print(" (END=");
      Serial.print(endValue);
      Serial.println(")");
      Serial.print("Firmware version: ");
      Serial.print(fwMajor);
      Serial.print(".");
      Serial.println(fwMinor);
      Serial.print("Servo version   : ");
      Serial.print(servoMajor);
      Serial.print(".");
      Serial.println(servoMinor);
      Serial.print("Model number    : ");
      Serial.print(modelNumber);
      Serial.print(" (0x");
      if (modelNumber < 0x1000)
        Serial.print("0");
      if (modelNumber < 0x100)
        Serial.print("0");
      if (modelNumber < 0x10)
        Serial.print("0");
      Serial.print(modelNumber, HEX);
      Serial.print(") -> ");
      Serial.println(modelName);
    }
    else
    {
      Serial.println("Invalid response received");
    }
  }
  else
  {
    Serial.println("No response from servo");
  }

  // 送信専用モードに確実に復帰
  busDirTx();
}

// パケット内容をHEXで表示するデバッグ関数
void debugPacketContent(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
  Serial.println("\n=== Packet Debug ===");

  u8 packet[MAX_PACKET_SIZE];
  u16 packetSize = buildSyncWritePosPacket(packet, ID, IDN, Position, Speed, ACC);

  // HEX表示
  Serial.print("Packet (");
  Serial.print(packetSize);
  Serial.print(" bytes): ");
  for (u16 i = 0; i < packetSize; i++)
  {
    if (packet[i] < 0x10)
      Serial.print("0");
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// [t]コマンド: 送信テスト（ブロードキャストで全IDにトルクONと位置指令を送る。受信不要でTX側の確認用）
void executeTxTestCommand()
{
  Serial.println("TX test: broadcasting torque ON and position to ALL IDs: +30, -30, +30, -30, center.");
  Serial.println("No reception is needed. If the servo moves, TX (ESP32 -> servo) is OK.");

  EnableTorque(0xFE, 1);
  delay(10);

  const float angles[] = {30.0, -30.0, 30.0, -30.0, 0.0};
  const bool isSCS = (servoType == SERVO_TYPE_SCS);
  for (u8 step = 0; step < sizeof(angles) / sizeof(angles[0]); step++)
  {
    // STS: アドレス41から ACC + 位置 + 時間 + 速度, SCS: アドレス42から 位置 + 時間 + 速度
    u8 data[7];
    u8 length = 0;
    if (!isSCS)
      data[length++] = 50; // 加速度
    encodeU16(angleToPosition(angles[step]), &data[length]);
    length += 2;
    encodeU16(0, &data[length]); // 時間（0固定）
    length += 2;
    encodeU16(1000, &data[length]); // 速度（[x]と同じ）
    length += 2;
    flushReceiveBuffer();
    busDirTx();
    sendPacket(0xFE, CMD_WRITE, isSCS ? SCS_GOAL_POSITION : SMS_STS_ACC, data, length);

    Serial.print("  ");
    Serial.print(angles[step], 0);
    Serial.println("deg sent");
    delay(700);
  }

  Serial.println("TX test finished (torque stays ON at center. Use [d] to release).");
  Serial.println("Moved     -> TX OK. If [f] finds nothing, check RX path and baud rate.");
  Serial.println("Not moved -> check power, wiring, and baud rate ([b] / [f]).");
}

void disp_menu()
{
  Serial.println("============================================");
  Serial.print("Current target ID: ");
  Serial.print(targetServoID);
  Serial.print(", Sync IDs: ");
  printSyncServoIDs();
  Serial.print(", Baud rate: ");
  Serial.print(STS_BAUD_RATES[servoBaudIndex]);
  Serial.print(" bps, Series: ");
  Serial.println(servoTypeName(servoType));
  Serial.println("Menu: [f] Find servos   [n] Set sync servo IDs   [z] Set target ID ");
  Serial.println("      [v] Read voltage  [r] Read position        [p] Read register");
  Serial.println("      [c] All to center [a] Read all positions   [s] Write register");
  Serial.println("      [e] All torque ON [d] All torque OFF       [x] Send angle to all ");
  Serial.println("      [b] Set baud rate [i] Change servo ID      [m] Read version");
  Serial.println("      [t] TX test (all IDs) [h] Change servo type (restart)");
  Serial.println("Input Command:");
}

// 初期化
void setup(u8 series)
{
  Serial.begin(115200);
  // サーボバス開始（8N1, エコー自動判定, 送信モードで待機）
  busBegin(&Serial1, RX1, TX1, PIN_EN, STS_BAUD_RATES[servoBaudIndex], SERIAL_8N1);

  delay(1000);
  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());

  Serial.println("=====================================");
  Serial.println("Feetech STS/SCS Servo Setter");
  Serial.println("Batch Transmission Version");
  Serial.println("=====================================");
  Serial.println("Communication method: SyncWrite (0x83)");
  Serial.println("Optimization: Single writeServo() call");
  Serial.println("Packet size: Calculated dynamically");
  Serial.println("=====================================");

  // サーボ系列は起動時のサーボ種類選択で決める（[f]で検出した場合はその結果に変わる）
  servoType = series;
  Serial.print("Servo series: ");
  Serial.println(servoTypeName(servoType));

  // サーボのトルクイネーブル
  Serial.println("Enabling torque for active servos...");
  EnableTorque(1, 1);
  delay(100);
  EnableTorque(2, 1);
  delay(100);

  // 初期位置設定
  Serial.println("Setting initial positions...");
  u8 initIDs[] = {1, 2, 3, 4, 5, 6};
  s16 c = positionCenter();
  s16 initPositions[] = {c, c, c, c, c, c};
  u16 initSpeeds[] = {300, 300, 300, 300, 300, 300};
  u8 initACCs[] = {30, 30, 30, 30, 30, 30};

  // デバッグ表示
  debugPacketContent(initIDs, 6, initPositions, initSpeeds, initACCs);

  SyncWritePos(initIDs, 6, initPositions, initSpeeds, initACCs);
  delay(2000); // 初期位置到達待ち
  disp_menu();
  lastMoveTime = millis();
}

void loop()
{
  // シリアル入力チェック
  if (Serial.available())
  {
    String command = Serial.readString();
    command.trim();

    if (command == "x")
    {
      executeAngleCommand();
    }
    else if (command == "r")
    {
      executeReadPositionCommand();
    }
    else if (command == "v")
    {
      executeReadVoltageCommand();
    }
    else if (command == "z")
    {
      executeChangeTargetIdCommand();
    }
    else if (command == "n")
    {
      executeSetSyncServoIDsCommand();
    }
    else if (command == "a")
    {
      executeReadAllPositionsCommand();
    }
    else if (command == "c")
    {
      executeResetAllPositionsCommand();
    }
    else if (command == "f")
    {
      executeServoDetectionCommand();
    }
    else if (command == "e")
    {
      executeEnableTorqueAllCommand();
    }
    else if (command == "d")
    {
      executeDisableTorqueAllCommand();
    }
    else if (command == "p")
    {
      executeReadRegisterCommand();
    }
    else if (command == "s")
    {
      executeWriteRegisterCommand();
    }
    else if (command == "i")
    {
      executeChangeServoIdCommand();
    }
    else if (command == "m")
    {
      executeReadVersionCommand();
    }
    else if (command == "b")
    {
      executeSetBaudRateCommand();
    }
    else if (command == "t")
    {
      executeTxTestCommand();
    }
    else if (command == "h")
    {
      selectServoTypeAndRestart();
    }
    else if (command.length() == 0 || command == "\r" || command == "\n")
    {
      disp_menu();
    }
    else if (command.length() > 0)
    {
      Serial.println("Unknown command. Available commands:");
      disp_menu();
    }
  }

  delay(50); // CPU負荷軽減
}

// ===== サーボ種類の自動判別（起動時のメニュー [0]） =====

// 読み取りコマンドだけでSTS / SCS のサーボを探す（サーボは動かない）。
// 系列（STS / SCS）はアドレス2（END）の値で判定する。
// defaultBaud: true = 工場出荷時の通信速度だけ, false = それ以外の速度。見つかった通信速度（なければ0）
unsigned long detect(bool defaultBaud, u8 *series)
{
  u8 endValue = SERVO_TYPE_STS;
  const u8 original = servoBaudIndex;
  unsigned long found = 0;
  busBegin(servoSerial, RX1, TX1, PIN_EN, STS_BAUD_RATES[DEFAULT_BAUD_INDEX], SERIAL_8N1);
  for (int b = STS_BAUD_COUNT - 1; b >= 0 && found == 0; b--)
  {
    if ((b == DEFAULT_BAUD_INDEX) != defaultBaud)
      continue;
    setHostBaudRate(b);
    for (int id = 0; id <= DEFAULT_SCAN_MAX_ID; id++)
    {
      if (readByteRegister(id, REG_END, &endValue, detectTimeout(STS_BAUD_RATES[b], 7)))
      {
        found = STS_BAUD_RATES[b];
        break;
      }
    }
  }
  servoBaudIndex = original;
  *series = (endValue == SERVO_TYPE_SCS) ? SERVO_TYPE_SCS : SERVO_TYPE_STS; // END: 0 = STS, 1 = SCS
  return found;
}

// 起動前に通信速度を設定する（自動判別で見つかった速度で始めるため）
void presetBaud(unsigned long baud)
{
  for (u8 i = 0; i < STS_BAUD_COUNT; i++)
  {
    if (STS_BAUD_RATES[i] == baud)
      servoBaudIndex = i;
  }
}

} // namespace feetech
