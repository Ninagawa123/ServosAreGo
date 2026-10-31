#include <Arduino.h>
#include "servo_bus.h"
#include "servo_select.h"

namespace futaba
{

// 基本型定義
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;

// ピン定義
#define PIN_EN 33 // 半二重通信の送受信切替（HIGH: 送信, LOW: 受信）

// Futaba RS30x パケット定義
#define TX_HEADER_1 0xFA // 送信パケットヘッダ
#define TX_HEADER_2 0xAF
#define RX_HEADER_1 0xFD // リターンパケットヘッダ
#define RX_HEADER_2 0xDF

// 送信フラグ定義
#define FLAG_NONE 0x00         // リターンなし
#define FLAG_RETURN_RANGE 0x0F // 指定アドレスから指定長を返信（アドレス0-59）
#define FLAG_FACTORY_RESET 0x10 // No.4-29を工場出荷値に戻す
#define FLAG_REBOOT 0x20       // 再起動
#define FLAG_FLASH_WRITE 0x40  // No.4-29をフラッシュROMに保存

// リターンパケットのステータスフラグ
#define STATUS_TEMP_ERROR 0x80  // 温度リミットエラー（トルクOFF）
#define STATUS_TEMP_ALARM 0x20  // 温度リミットアラーム
#define STATUS_FLASH_ERROR 0x08 // フラッシュROM書き込みエラー
#define STATUS_PACKET_ERROR 0x02 // 受信パケット処理不可

// ID定義
#define BROADCAST_ID 0xFF // 全サーボ（リターンなし）
#define LONG_PACKET_ID 0x00
#define FUTABA_ID_MIN 1
#define FUTABA_ID_MAX 127

// メモリマップ定義（RS303MR / RS304MD）
#define ADDR_MODEL_NUMBER 0      // 2バイト
#define ADDR_FIRMWARE_VERSION 2  // 1バイト
#define ADDR_SERVO_ID 4          // 1バイト（ROM, 即時反映）
#define ADDR_REVERSE 5           // 1バイト（ROM）
#define ADDR_BAUD_RATE 6         // 1バイト（ROM, 再起動後に反映）
#define ADDR_RETURN_DELAY 7      // 1バイト（ROM, 100us + 値x50us）
#define ADDR_GOAL_POSITION 30    // 2バイト（0.1度, 符号付き）
#define ADDR_GOAL_TIME 32        // 2バイト（10ms）
#define ADDR_TORQUE_ENABLE 36    // 1バイト（0: OFF, 1: ON, 2: ブレーキ）
#define ADDR_PRESENT_POSITION 42 // 2バイト（0.1度, 符号付き）
#define ADDR_PRESENT_CURRENT 48  // 2バイト（mA）
#define ADDR_PRESENT_TEMP 50     // 2バイト（℃）
#define ADDR_PRESENT_VOLTS 52    // 2バイト（10mV）
#define ADDR_ROM_FIRST 4         // フラッシュ保存対象の先頭
#define ADDR_ROM_LAST 29         // フラッシュ保存対象の末尾
#define ADDR_READ_MAX 59         // 読み取り可能な最終アドレス

// 位置・角度定義
#define ANGLE_LIMIT 150.0      // 可動範囲 ±150度
#define POSITION_DEAD_ZONE 5555 // 不感帯検出時の現在位置（555.5度）

// 一斉移動の時間（10ms単位）
#define MOVE_GOAL_TIME 100   // 1.0秒
#define CENTER_GOAL_TIME 150 // 1.5秒

// 待ち時間定義（マニュアルに規定がないため参考コードに合わせ余裕を持たせる）
#define FLASH_WRITE_WAIT 300 // フラッシュ書き込み後の待ち(ms)
#define REBOOT_WAIT 1000     // 再起動後の待ち(ms)

// パケット・リスト定義
#define MAX_PACKET_SIZE 1024
#define MAX_WRITE_DATA 16
#define MAX_SYNC_SERVOS 127 // 一斉制御リストの最大数（有効ID 1-127）

// 通信タイムアウト定義
#define RESPONSE_TIMEOUT 100 // 通常の応答待ち(ms)
#define SCAN_TIMEOUT 30      // サーボ検索時の応答待ち(ms)

// サーボ検索のデフォルト最大ID（[f]で空欄リターン時）
#define DEFAULT_SCAN_MAX_ID 20

// Futaba通信速度（インデックス = ボーレートレジスタ値）。0x0A, 0x0Bはメーカー保証外
const unsigned long FUTABA_BAUD_RATES[] = {9600, 14400, 19200, 28800, 38400, 57600,
                                           76800, 115200, 153600, 230400, 460800, 691200};
#define FUTABA_BAUD_COUNT (sizeof(FUTABA_BAUD_RATES) / sizeof(FUTABA_BAUD_RATES[0]))
#define OFFICIAL_BAUD_MAX 9  // 公式対応は0x09（230400bps）まで
#define DEFAULT_BAUD_INDEX 7 // 115200bps（工場出荷値）

// 既知のサーボ型番（取扱説明書 RS303MR/RS304MD Ver.1.18 より）
struct FutabaModel
{
  u16 number;
  const char *name;
};
const FutabaModel FUTABA_MODELS[] = {
    {0x3030, "RS303MR"},
    {0x3040, "RS304MD"},
};
#define FUTABA_MODEL_COUNT (sizeof(FUTABA_MODELS) / sizeof(FUTABA_MODELS[0]))

// 通信結果
enum RsResult
{
  RS_OK,
  RS_NO_RESPONSE,
  RS_INVALID_RESPONSE
};

// グローバル変数
HardwareSerial *servoSerial = &Serial1;                 // 使用するシリアルポート
u8 targetServoID = 1;                                   // ターゲットサーボID
u8 syncServoIDs[MAX_SYNC_SERVOS] = {1, 2, 3, 4, 5, 6}; // 一斉制御するサーボIDリスト（とびとびIDも可）
u8 syncServoCount = 6;                                  // リスト内のサーボ数
u8 servoBaudIndex = DEFAULT_BAUD_INDEX;                 // ESP32側の現在の通信速度インデックス
u8 lastStatusFlags = 0;                                 // 最後に受信したリターンパケットのステータス

// 送信バッファ
u8 txPacket[MAX_PACKET_SIZE];

// ===== 共通ユーティリティ =====

// シリアル入力を1行読み取り（前後の空白除去）
String readInput()
{
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();
  return input;
}

// 数字のみで構成された入力か（先頭の'-'は許可オプション）
bool isNumber(const String &input, bool allowNegative)
{
  if (input.length() == 0)
    return false;
  for (unsigned int i = 0; i < input.length(); i++)
  {
    if (i == 0 && allowNegative && input[i] == '-' && input.length() > 1)
      continue;
    if (!isDigit(input[i]))
      return false;
  }
  return true;
}

// 型番名（未知の型番は"Unknown"）
const char *modelName(u16 modelNumber)
{
  for (u8 i = 0; i < FUTABA_MODEL_COUNT; i++)
  {
    if (FUTABA_MODELS[i].number == modelNumber)
      return FUTABA_MODELS[i].name;
  }
  return "Unknown";
}

// 通信結果の表示用テキスト
const char *rsResultText(RsResult result)
{
  switch (result)
  {
  case RS_OK:
    return "OK";
  case RS_NO_RESPONSE:
    return "No response";
  default:
    return "Invalid response (format/checksum)";
  }
}

// ステータスフラグの内容表示
void printStatusFlags(u8 flags)
{
  if (flags == 0)
  {
    Serial.print("None");
    return;
  }
  Serial.print("0x");
  Serial.print(flags, HEX);
  if (flags & STATUS_TEMP_ERROR)
    Serial.print(" TempLimitError(torque OFF)");
  if (flags & STATUS_TEMP_ALARM)
    Serial.print(" TempLimitAlarm");
  if (flags & STATUS_FLASH_ERROR)
    Serial.print(" FlashWriteError");
  if (flags & STATUS_PACKET_ERROR)
    Serial.print(" PacketError");
}

// ===== RS30x 通信レイヤー =====

// ショートパケット構築（チェックサム = IDからデータ末尾までのXOR）。戻り値はパケット長
u16 buildShortPacket(u8 *packet, u8 id, u8 flags, u8 address, u8 length, u8 count, const u8 *data, u8 dataLength)
{
  u16 index = 0;

  // ヘッダ構築
  packet[index++] = TX_HEADER_1;
  packet[index++] = TX_HEADER_2;
  packet[index++] = id;
  packet[index++] = flags;
  packet[index++] = address;
  packet[index++] = length;
  packet[index++] = count;

  // データ追加
  for (u8 i = 0; i < dataLength; i++)
  {
    packet[index++] = data[i];
  }

  // チェックサム追加
  u8 sum = 0;
  for (u16 i = 2; i < index; i++)
  {
    sum ^= packet[i];
  }
  packet[index++] = sum;

  return index;
}

// 受信バッファフラッシュ
void flushReceiveBuffer()
{
  while (servoSerial->read() != -1)
    ;
}

// 半二重通信でパケット送信（送信後も送信モードのまま）
void sendPacket(const u8 *packet, u16 length)
{
  // 受信バッファクリア → 送信
  flushReceiveBuffer();
  busDirTx();
  servoSerial->write(packet, length);
  servoSerial->flush();
  delayMicroseconds(50);
}

// 半二重通信でパケット送信し、送信完了後すぐ受信モードに切り替える（送信エコーは読み捨て）
void sendPacketToRx(const u8 *packet, u16 length)
{
  flushReceiveBuffer();
  busDirTx();
  servoSerial->write(packet, length);
  busEndSendToRx();
  busSkipEcho(packet, length, 10);
}

// リターンパケット受信（FD DF ID Flags Addr Len Cnt Data Sum）
RsResult receiveReturn(u8 id, u8 address, u8 *data, u8 length, unsigned long timeout)
{
  u8 header[5]; // ID Flags Addr Len Cnt
  u16 index = 0; // 0-1: ヘッダ, 2-6: header[], 7-: データ, 最後: チェックサム
  u16 expectedLength = 7 + length + 1;
  unsigned long startTime = millis();

  while ((millis() - startTime) < timeout && index < expectedLength)
  {
    if (!servoSerial->available())
      continue;

    u8 value = servoSerial->read();

    // ヘッダ（FD DF）同期
    if (index == 0)
    {
      if (value == RX_HEADER_1)
        index = 1;
      continue;
    }
    if (index == 1)
    {
      if (value == RX_HEADER_2)
        index = 2;
      else if (value != RX_HEADER_1)
        index = 0;
      continue;
    }

    if (index < 7)
      header[index - 2] = value;
    else if (index < 7 + length)
      data[index - 7] = value;
    else
    {
      // チェックサム検証（IDからデータ末尾までのXOR）
      u8 sum = 0;
      for (u8 i = 0; i < 5; i++)
        sum ^= header[i];
      for (u8 i = 0; i < length; i++)
        sum ^= data[i];
      if (sum != value)
        return RS_INVALID_RESPONSE;
    }
    index++;
  }

  if (index < expectedLength)
    return (index == 0) ? RS_NO_RESPONSE : RS_INVALID_RESPONSE;

  // ID・アドレス・長さ検証
  if (header[0] != id || header[2] != address || header[3] != length)
    return RS_INVALID_RESPONSE;

  lastStatusFlags = header[1];
  return RS_OK;
}

// 読み取り（Flags=0x0F: 指定アドレスから指定長を返信。アドレス0-59）
RsResult rsRead(u8 id, u8 address, u8 length, u8 *data, unsigned long timeout = RESPONSE_TIMEOUT)
{
  if (address + length - 1 > ADDR_READ_MAX)
    return RS_INVALID_RESPONSE;

  u16 packetLength = buildShortPacket(txPacket, id, FLAG_RETURN_RANGE, address, length, 0, NULL, 0);
  // 送信 → すぐ受信モードに切り替え
  sendPacketToRx(txPacket, packetLength);
  RsResult result = receiveReturn(id, address, data, length, timeout);

  // 送信モードに復帰
  busDirTx();

  // 温度・フラッシュ異常の通知
  if (result == RS_OK && (lastStatusFlags & (STATUS_TEMP_ERROR | STATUS_TEMP_ALARM | STATUS_FLASH_ERROR)))
  {
    Serial.print("[Alert] ID ");
    Serial.print(id);
    Serial.print(": ");
    printStatusFlags(lastStatusFlags);
    Serial.println();
  }

  return result;
}

// 書き込み（ショートパケット, リターンなし）
void rsWrite(u8 id, u8 address, const u8 *data, u8 length)
{
  u16 packetLength = buildShortPacket(txPacket, id, FLAG_NONE, address, length, 1, data, length);
  sendPacket(txPacket, packetLength);
}

// 1/2バイト値の読み取り（2バイトはリトルエンディアン, signedで符号付き）
RsResult rsReadValue(u8 id, u8 address, u8 size, long *value, bool isSigned, unsigned long timeout = RESPONSE_TIMEOUT)
{
  u8 data[2] = {0, 0};
  RsResult result = rsRead(id, address, size, data, timeout);
  if (result == RS_OK)
  {
    if (size == 1)
      *value = data[0];
    else
      *value = isSigned ? (long)(s16)(data[0] | (data[1] << 8)) : (long)(u16)(data[0] | (data[1] << 8));
  }
  return result;
}

// 1/2バイト値の書き込み（2バイトはリトルエンディアン）
void rsWriteValue(u8 id, u8 address, u8 size, long value)
{
  u8 data[2] = {(u8)(value & 0xFF), (u8)((value >> 8) & 0xFF)};
  rsWrite(id, address, data, size);
}

// フラッシュROM書き込み（No.4-29を保存。書き込み中は電源を切らないこと）
void rsFlashWrite(u8 id)
{
  u16 packetLength = buildShortPacket(txPacket, id, FLAG_FLASH_WRITE, 0xFF, 0, 0, NULL, 0);
  sendPacket(txPacket, packetLength);
  delay(FLASH_WRITE_WAIT);
}

// 再起動（フラッシュ書き込み完了後に別パケットで送ること）
void rsReboot(u8 id)
{
  u16 packetLength = buildShortPacket(txPacket, id, FLAG_REBOOT, 0xFF, 0, 0, NULL, 0);
  sendPacket(txPacket, packetLength);
  delay(REBOOT_WAIT);
}

// 工場出荷リセット（No.4-29。マニュアル記載例どおりデータなし）
void rsFactoryReset(u8 id)
{
  u16 packetLength = buildShortPacket(txPacket, id, FLAG_FACTORY_RESET, 0xFF, 0xFF, 0, NULL, 0);
  sendPacket(txPacket, packetLength);
  delay(FLASH_WRITE_WAIT);
}

// ロングパケット（同一アドレス・同一長を複数サーボへ, リターンなし）
void rsLongWrite(u8 address, u8 dataLength, const u8 *ids, u8 count, const u8 *data)
{
  static u8 body[MAX_PACKET_SIZE];
  u16 index = 0;
  for (u8 i = 0; i < count; i++)
  {
    body[index++] = ids[i];
    memcpy(&body[index], &data[i * dataLength], dataLength);
    index += dataLength;
  }

  u16 packetLength = buildShortPacket(txPacket, LONG_PACKET_ID, FLAG_NONE, address, 1 + dataLength, count, body, index);
  sendPacket(txPacket, packetLength);
}

// PING相当（型番とファームウェア版を読み取って存在確認）
RsResult rsPing(u8 id, u16 *modelNumber, u8 *firmwareVersion, unsigned long timeout)
{
  u8 data[3];
  RsResult result = rsRead(id, ADDR_MODEL_NUMBER, 3, data, timeout);
  if (result == RS_OK)
  {
    if (modelNumber)
      *modelNumber = data[0] | (data[1] << 8);
    if (firmwareVersion)
      *firmwareVersion = data[2];
  }
  return result;
}

// ESP32側のサーボ通信速度を変更
void setHostBaudRate(u8 baudIndex)
{
  servoSerial->flush();
  servoSerial->updateBaudRate(FUTABA_BAUD_RATES[baudIndex]);
  servoBaudIndex = baudIndex;
  delay(10);
  flushReceiveBuffer();
}

// ===== 一斉制御リスト =====

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
  bool selected[FUTABA_ID_MAX + 1] = {false};
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

    // 数字と'-'（1個まで、先頭・末尾以外）以外は不正
    int dashCount = 0;
    for (unsigned int k = 0; k < token.length(); k++)
    {
      if (token[k] == '-')
        dashCount++;
      else if (!isDigit(token[k]))
        return -1;
    }
    int dash = token.indexOf('-');
    if (dashCount > 1 || dash == 0 || dash == (int)token.length() - 1)
      return -1;

    // 範囲指定（a-b）または単一ID
    int first = token.toInt();
    int last = dash >= 0 ? token.substring(dash + 1).toInt() : first;
    if (first < FUTABA_ID_MIN || last > FUTABA_ID_MAX || first > last)
      return -1;

    for (int id = first; id <= last; id++)
      selected[id] = true;
  }

  int count = 0;
  for (int id = FUTABA_ID_MIN; id <= FUTABA_ID_MAX; id++)
  {
    if (selected[id])
      out[count++] = id;
  }
  return count > 0 ? count : -1;
}

// 一斉制御リストの全サーボを指定位置へ移動（トルクON → 目標位置 + 時間）
void moveSyncServos(s16 position, u16 goalTime)
{
  static u8 data[MAX_SYNC_SERVOS * 4];

  // トルクOFF中の角度指令は無視されるため、先にトルクONを別パケットで送る
  for (u8 i = 0; i < syncServoCount; i++)
    data[i] = 1;
  rsLongWrite(ADDR_TORQUE_ENABLE, 1, syncServoIDs, syncServoCount, data);
  delay(10);

  // Goal Position(30) + Goal Time(32) を一括書き込み
  for (u8 i = 0; i < syncServoCount; i++)
  {
    data[i * 4] = position & 0xFF;
    data[i * 4 + 1] = (position >> 8) & 0xFF;
    data[i * 4 + 2] = goalTime & 0xFF;
    data[i * 4 + 3] = (goalTime >> 8) & 0xFF;
  }

  Serial.print("LongPacket: ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.print("), Pos=");
  Serial.print(position / 10.0, 1);
  Serial.print("deg (");
  Serial.print(position);
  Serial.print("), Time=");
  Serial.print(goalTime * 10);
  Serial.println("ms");

  rsLongWrite(ADDR_GOAL_POSITION, 4, syncServoIDs, syncServoCount, data);
}

// ROM領域の変更をフラッシュ保存して読み返し確認
bool saveAndVerify(u8 id, u8 address, long expected)
{
  rsFlashWrite(id);

  long value;
  RsResult result = rsReadValue(id, address, 1, &value, false);
  if (result != RS_OK)
  {
    Serial.print("Verification failed: ");
    Serial.println(rsResultText(result));
    return false;
  }
  if (lastStatusFlags & STATUS_FLASH_ERROR)
  {
    Serial.println("[Error] Flash ROM write error reported by servo");
    return false;
  }

  Serial.print("Verification: Read value = ");
  Serial.print(value);
  Serial.println(value == expected ? " [OK] (saved to flash ROM)" : " [MISMATCH]");
  return value == expected;
}

// ===== コマンド =====

// 指定範囲のIDを検索（現在のESP32通信速度）。見つかった数を返す
u8 scanServos(int maxId, u8 *foundServos, u16 *foundModels, bool showProgress)
{
  u8 foundCount = 0;

  // プログレス表示用
  int progressStep = maxId / 10;
  if (progressStep == 0)
    progressStep = 1;

  for (int id = FUTABA_ID_MIN; id <= maxId; id++)
  {
    // プログレス表示
    if (showProgress && (id % progressStep == 0 || id == maxId))
    {
      Serial.print("Progress: ");
      Serial.print((id * 100) / maxId);
      Serial.println("%");
    }

    u16 modelNumber;
    u8 firmwareVersion;
    if (rsPing(id, &modelNumber, &firmwareVersion, SCAN_TIMEOUT) == RS_OK)
    {
      foundServos[foundCount] = id;
      foundModels[foundCount] = modelNumber;
      foundCount++;

      Serial.print("Found servo ID: ");
      Serial.print(id);
      Serial.print(" (");
      Serial.print(modelName(modelNumber));
      Serial.print(", model 0x");
      Serial.print(modelNumber, HEX);
      Serial.print(", FW ");
      Serial.print(firmwareVersion);
      Serial.println(")");
    }

    delay(2); // 次のスキャンまで少し待機
  }

  return foundCount;
}

// 検索結果を表示し、ターゲットIDと一斉制御リストを自動設定
void applyScanResult(int maxId, const u8 *foundServos, const u16 *foundModels, u8 foundCount)
{
  Serial.println();
  Serial.println("=== Servo Detection Results ===");
  Serial.print("Scanned range: ID1 to ID");
  Serial.print(maxId);
  Serial.print(" at ");
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  Serial.print("Found ");
  Serial.print(foundCount);
  Serial.println(" servo(s)");

  Serial.print("Connected IDs: ");
  for (u8 i = 0; i < foundCount; i++)
  {
    Serial.print(foundServos[i]);
    Serial.print("(");
    Serial.print(modelName(foundModels[i]));
    Serial.print(")");
    if (i < foundCount - 1)
      Serial.print(", ");
  }
  Serial.println();

  // 見つかったIDにターゲットとリストを自動設定
  memcpy(syncServoIDs, foundServos, foundCount);
  syncServoCount = foundCount;
  targetServoID = syncServoIDs[0];

  Serial.print("Target ID auto-set to: ");
  Serial.println(targetServoID);
  Serial.print("Sync servo IDs auto-set to: ");
  printSyncServoIDs();
  Serial.println();
  Serial.println();
}

// 全ボーレートを速い順に検索し、最も多く見つかった速度にESP32を設定
void scanAllBaudRates(int maxId)
{
  static u8 ids[MAX_SYNC_SERVOS];
  static u16 models[MAX_SYNC_SERVOS];
  static u8 bestIds[MAX_SYNC_SERVOS];
  static u16 bestModels[MAX_SYNC_SERVOS];

  u8 originalIndex = servoBaudIndex;
  int bestIndex = -1;
  u8 bestCount = 0;
  u8 baudsWithServos = 0;

  Serial.println();
  Serial.println("=== Scanning all baud rates (fastest first) ===");

  for (int b = FUTABA_BAUD_COUNT - 1; b >= 0; b--)
  {
    setHostBaudRate(b);
    Serial.print("Scanning at ");
    Serial.print(FUTABA_BAUD_RATES[b]);
    Serial.println(" bps...");

    u8 count = scanServos(maxId, ids, models, false);

    Serial.print("  -> ");
    Serial.print(count);
    Serial.println(" servo(s)");

    if (count > 0)
      baudsWithServos++;

    // 同数の場合は先に見つかった（速い）方を優先
    if (count > bestCount)
    {
      bestCount = count;
      bestIndex = b;
      memcpy(bestIds, ids, count);
      memcpy(bestModels, models, count * sizeof(u16));
    }
  }

  if (bestIndex < 0)
  {
    setHostBaudRate(originalIndex);
    Serial.println();
    Serial.println("No servos detected at any baud rate");
    Serial.println("Hint: Check wiring/power. Use [b] -> [3] (MADI write) to recover the baud rate");
    Serial.println();
    return;
  }

  setHostBaudRate(bestIndex);
  Serial.println();
  Serial.print("ESP32 baud rate auto-set to: ");
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");

  if (baudsWithServos > 1)
  {
    Serial.println("[Warning] Servos were found at multiple baud rates.");
    Serial.println("          Using the baud rate with the most servos. Use [b] to unify them.");
  }

  applyScanResult(maxId, bestIds, bestModels, bestCount);
}

// 現在の通信速度で ID1〜maxId を検索し、見つかればターゲットとリストを自動設定。見つかった数を返す
u8 scanAndApply(int maxId, u8 *foundServos, u16 *foundModels)
{
  Serial.print("Scanning servos from ID1 to ID");
  Serial.print(maxId);
  Serial.print(" at ");
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps...");
  Serial.println();

  u8 foundCount = scanServos(maxId, foundServos, foundModels, true);
  if (foundCount > 0)
    applyScanResult(maxId, foundServos, foundModels, foundCount);
  return foundCount;
}

// [f]コマンド: サーボ認識（見つからなければ全ボーレート検索を提案）
void executeServoDetectionCommand()
{
  static u8 foundServos[MAX_SYNC_SERVOS];
  static u16 foundModels[MAX_SYNC_SERVOS];

  Serial.print("Enter maximum ID to scan (1-127, empty = ");
  Serial.print(DEFAULT_SCAN_MAX_ID);
  Serial.println("): ");

  String input = readInput();

  // 空欄リターンはデフォルト最大IDで検索
  int maxId = input.length() == 0 ? DEFAULT_SCAN_MAX_ID : input.toInt();

  // ID範囲チェック
  if ((input.length() > 0 && !isNumber(input, false)) || maxId < FUTABA_ID_MIN || maxId > FUTABA_ID_MAX)
  {
    Serial.println("Error: ID out of range (1-127)");
    return;
  }

  if (scanAndApply(maxId, foundServos, foundModels) > 0)
    return;

  Serial.println();
  Serial.print("No servos detected at ");
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  Serial.println("Press Enter to scan all baud rates (fastest first), or type any key + Enter to cancel:");

  if (readInput().length() == 0)
  {
    scanAllBaudRates(maxId);
  }
  else
  {
    Serial.println("Canceled");
    Serial.println();
  }
}

// [z]コマンド: ターゲットID変更
void executeChangeTargetIdCommand()
{
  Serial.print("Current target ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter new target ID (1-127): ");

  String input = readInput();
  int newId = input.toInt();

  // ID範囲チェック → 設定
  if (!isNumber(input, false) || !apiSetTarget(newId))
  {
    Serial.println("Error: ID out of range (1-127)");
    return;
  }

  Serial.print("Target ID changed to: ");
  Serial.println(targetServoID);
}

// [n]コマンド: 一斉制御サーボIDリスト設定
void executeSetSyncServoIDsCommand()
{
  Serial.print("Current sync servo IDs: ");
  printSyncServoIDs();
  Serial.println();
  Serial.println("Enter sync servo IDs (e.g. 1,3,7 or 1-4,7): ");

  String input = readInput();

  u8 newIDs[MAX_SYNC_SERVOS];
  int newCount = parseIdList(input, newIDs);
  if (newCount < 0 || !apiSetSync(newIDs, newCount))
  {
    Serial.println("Error: Invalid ID list (valid IDs: 1-127)");
    return;
  }

  Serial.print("Sync servo IDs changed to: ");
  printSyncServoIDs();
  Serial.print(" (");
  Serial.print(syncServoCount);
  Serial.println(" servos)");
}

// [r]コマンド: サーボ位置読み取り
void executeReadPositionCommand()
{
  float deg;
  apiReadPosition(targetServoID, &deg);
}

// [v]コマンド: サーボ電圧・電流・温度読み取り
void executeReadVoltageCommand()
{
  Serial.print("Reading voltage from servo ID ");
  Serial.println(targetServoID);

  // Present Current(48) + Temperature(50) + Volts(52) を一括読み取り
  u8 data[6];
  RsResult result = rsRead(targetServoID, ADDR_PRESENT_CURRENT, 6, data);
  if (result != RS_OK)
  {
    Serial.print("Error: ");
    Serial.println(rsResultText(result));
    return;
  }

  u16 current = data[0] | (data[1] << 8);
  u16 temperature = data[2] | (data[3] << 8);
  u16 volts = data[4] | (data[5] << 8);

  Serial.print("Voltage    : ");
  Serial.print(volts);
  Serial.print(" (");
  Serial.print(volts / 100.0, 2);
  Serial.println("V)");
  Serial.print("Current    : ");
  Serial.print(current);
  Serial.println(" mA (approx.)");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
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
    long position;
    RsResult result = rsReadValue(id, ADDR_PRESENT_POSITION, 2, &position, true);

    Serial.print(id);
    if (result == RS_OK)
    {
      Serial.print("\t");
      Serial.print(position);
      Serial.print("\t\t");
      Serial.print(position / 10.0, 1);
      Serial.println(position == POSITION_DEAD_ZONE ? "° [Dead zone]" : "°");
      connectedCount++;
    }
    else
    {
      Serial.print("\t[Error] ");
      Serial.println(rsResultText(result));
    }
  }

  Serial.println("-------------------------");
  Serial.print("Connected servos: ");
  Serial.print(connectedCount);
  Serial.print("/");
  Serial.println(syncServoCount);
}

// [c]コマンド: 全サーボ位置を中央にリセット
void executeResetAllPositionsCommand()
{
  apiCenter();
}

// [x]コマンド: 全サーボに角度を一斉送信
void executeAngleCommand()
{
  Serial.println("Enter angle (-150 to 150 degrees): ");

  String input = readInput();
  float angle = input.toFloat();

  // 角度範囲チェック → 一斉送信
  if (input.length() == 0 || !apiMove(angle))
  {
    Serial.println("Error: Angle out of range (-150 to 150)");
  }
}

// [e]/[d]コマンド共通: 全サーボのトルク切替（読み返しで成否確認）。全台成功で true
bool setTorqueAll(u8 enable)
{
  Serial.print(enable ? "Enabling" : "Disabling");
  Serial.print(" torque for ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");

  u8 successCount = 0;

  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    rsWriteValue(id, ADDR_TORQUE_ENABLE, 1, enable);
    delay(5);

    long value;
    RsResult result = rsReadValue(id, ADDR_TORQUE_ENABLE, 1, &value, false);

    Serial.print("ID ");
    Serial.print(id);
    if (result != RS_OK)
    {
      Serial.print(": Failed (");
      Serial.print(rsResultText(result));
      Serial.println(")");
    }
    else if (value != enable)
    {
      Serial.print(": Failed (Torque Enable = ");
      Serial.print(value);
      Serial.println(value == 2 ? ", brake mode after temperature error)" : ", shaft may be in dead zone)");
    }
    else
    {
      Serial.println(enable ? ": Torque enabled" : ": Torque disabled");
      successCount++;
    }
  }

  Serial.print(enable ? "Torque enabled for " : "Torque disabled for ");
  Serial.print(successCount);
  Serial.print("/");
  Serial.print(syncServoCount);
  Serial.println(" servos");
  return successCount == syncServoCount;
}

// [p]コマンド: 指定アドレスのレジスタ値読み取り
void executeReadRegisterCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter register address (0-59): ");

  String addressInput = readInput();
  int address = addressInput.toInt();

  // アドレス範囲チェック
  if (!isNumber(addressInput, false) || address < 0 || address > ADDR_READ_MAX)
  {
    Serial.println("Error: Address out of range (0-59)");
    return;
  }

  Serial.println("Enter data length to read (1 or 2 bytes): ");

  int dataLength = readInput().toInt();

  // データ長チェック
  if ((dataLength != 1 && dataLength != 2) || address + dataLength - 1 > ADDR_READ_MAX)
  {
    Serial.println("Error: Data length must be 1 or 2 (within address 0-59)");
    return;
  }

  Serial.print("Reading ");
  Serial.print(dataLength);
  Serial.print(" byte(s) from address ");
  Serial.print(address);
  Serial.print(" of servo ID ");
  Serial.println(targetServoID);

  long value;
  RsResult result = rsReadValue(targetServoID, address, dataLength, &value, false);
  if (result != RS_OK)
  {
    Serial.print("Error: ");
    Serial.println(rsResultText(result));
    return;
  }

  Serial.print("Register ");
  Serial.print(address);
  Serial.print(" (0x");
  if (address < 16)
    Serial.print("0");
  Serial.print(address, HEX);
  Serial.print("): Dec=");
  Serial.print(value);
  if (dataLength == 2)
  {
    Serial.print(" (signed ");
    Serial.print((s16)value);
    Serial.print(")");
  }
  Serial.print(", Hex=0x");
  Serial.print(value, HEX);
  Serial.print(", Bin=");
  Serial.println(value, BIN);
}

// [s]コマンド: 指定アドレスにレジスタ値書き込み（書き込み後に読み返して確認）
void executeWriteRegisterCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter register address (4-59): ");

  String addressInput = readInput();
  int address = addressInput.toInt();

  // アドレス範囲チェック（0-3は書き込み不可）
  if (!isNumber(addressInput, false) || address < ADDR_ROM_FIRST || address > ADDR_READ_MAX)
  {
    Serial.println("Error: Address out of range (4-59)");
    return;
  }

  // ID・ボーレートは専用コマンドで変更（検証・再起動の手順が異なるため）
  if (address == ADDR_SERVO_ID || address == ADDR_BAUD_RATE)
  {
    Serial.println("Error: Use [i] to change ID, [b] to change baud rate");
    return;
  }

  Serial.println("Enter data length to write (1 or 2 bytes): ");

  int dataLength = readInput().toInt();

  // データ長チェック
  if ((dataLength != 1 && dataLength != 2) || address + dataLength - 1 > ADDR_READ_MAX)
  {
    Serial.println("Error: Data length must be 1 or 2 (within address 4-59)");
    return;
  }

  Serial.print("Enter decimal value to write (");
  Serial.print(dataLength == 1 ? "0-255" : "-32768 to 65535");
  Serial.println("): ");

  String valueInput = readInput();
  long value = valueInput.toInt();

  // 値範囲チェック
  if (!isNumber(valueInput, true) ||
      (dataLength == 1 && (value < 0 || value > 255)) ||
      (dataLength == 2 && (value < -32768 || value > 65535)))
  {
    Serial.println("Error: Value out of range");
    return;
  }

  Serial.print("Writing ");
  Serial.print(value);
  Serial.print(" to address ");
  Serial.print(address);
  Serial.print(" of servo ID ");
  Serial.println(targetServoID);

  rsWriteValue(targetServoID, address, dataLength, value);
  Serial.println("Write command sent successfully");
  delay(10);

  // 確認のため値を読み返す
  long readValue;
  RsResult result = rsReadValue(targetServoID, address, dataLength, &readValue, false);
  if (result != RS_OK)
  {
    Serial.print("Verification failed: ");
    Serial.println(rsResultText(result));
    return;
  }

  long expected = (dataLength == 1) ? (value & 0xFF) : (value & 0xFFFF);
  Serial.print("Verification: Read value = ");
  Serial.print(readValue);
  Serial.println(readValue == expected ? " [OK]" : " [MISMATCH]");

  // ROM領域は電源OFFで消えるため、フラッシュ保存を確認
  if (address <= ADDR_ROM_LAST && readValue == expected)
  {
    Serial.println("This is ROM area. Save to flash ROM? (y = save, other = RAM only): ");
    if (readInput() == "y")
    {
      rsFlashWrite(targetServoID);
      Serial.println("Saved to flash ROM");
    }
    else
    {
      Serial.println("Not saved (will be lost at power off)");
    }
  }
}

// ボーレート一覧表示
void printBaudRateList()
{
  Serial.println("Futaba RS30x baud rates:");
  for (u8 i = 0; i < FUTABA_BAUD_COUNT; i++)
  {
    Serial.print("  [");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(FUTABA_BAUD_RATES[i]);
    Serial.print(" bps");
    if (i == DEFAULT_BAUD_INDEX)
      Serial.print(" (default)");
    if (i > OFFICIAL_BAUD_MAX)
      Serial.print(" (unofficial)");
    Serial.println();
  }
}

// 全ボーレートでブロードキャスト書き換え（MADI write: 速度不明のサーボも復旧可能）
void madiWriteBaudRate(u8 baudIndex)
{
  Serial.print("MADI write: setting all servos to ");
  Serial.print(FUTABA_BAUD_RATES[baudIndex]);
  Serial.println(" bps at every baud rate...");

  for (int b = FUTABA_BAUD_COUNT - 1; b >= 0; b--)
  {
    setHostBaudRate(b);
    rsWriteValue(BROADCAST_ID, ADDR_BAUD_RATE, 1, baudIndex);
    delay(10);
    rsFlashWrite(BROADCAST_ID);
    rsReboot(BROADCAST_ID);

    Serial.print("  ");
    Serial.print(FUTABA_BAUD_RATES[b]);
    Serial.println(" bps: done");
  }

  setHostBaudRate(baudIndex);
  Serial.print("ESP32 baud rate changed to: ");
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  Serial.println("Use [f] to find servos");
}

// ESP32側だけ通信速度を変更して表示
void applyHostBaudRate(u8 baudIndex)
{
  setHostBaudRate(baudIndex);
  Serial.print("ESP32 baud rate changed to: ");
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
}

// ターゲットサーボの通信速度を変更し（フラッシュ保存 + 再起動）、ESP32も追従。新速度で読み返して確認
bool writeServoBaudRate(u8 baudIndex)
{
  // 変更元サーボの存在確認
  if (rsPing(targetServoID, NULL, NULL, RESPONSE_TIMEOUT) != RS_OK)
  {
    Serial.println("Error: No response from target servo (use [z] to select ID)");
    return false;
  }

  Serial.print("Writing baud rate ");
  Serial.print(FUTABA_BAUD_RATES[baudIndex]);
  Serial.print(" bps to servo ID ");
  Serial.println(targetServoID);

  // データ書き込み → フラッシュ保存 → 再起動（それぞれ別パケット。再起動後に新速度へ）
  rsWriteValue(targetServoID, ADDR_BAUD_RATE, 1, baudIndex);
  delay(10);
  rsFlashWrite(targetServoID);
  rsReboot(targetServoID);

  // ESP32側も新速度へ追従
  setHostBaudRate(baudIndex);

  // 新しい速度で読み返して確認
  Serial.println("Verifying at new baud rate...");
  bool ok = false;
  long readValue;
  RsResult result = rsReadValue(targetServoID, ADDR_BAUD_RATE, 1, &readValue, false);
  if (result == RS_OK)
  {
    ok = (readValue == baudIndex);
    Serial.print("Verification: Baud rate register = ");
    Serial.print(readValue);
    Serial.println(ok ? " [OK]" : " [MISMATCH]");
  }
  else
  {
    Serial.println("Verification failed: No response at new baud rate");
    Serial.println("Hint: Use [f] -> Enter to scan all baud rates, or [b] -> [3] (MADI write)");
  }

  Serial.print("ESP32 baud rate: ");
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  return ok;
}

// [b]コマンド: 通信速度設定
void executeSetBaudRateCommand()
{
  Serial.print("Current ESP32 baud rate: ");
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  printBaudRateList();
  Serial.print("Enter baud rate number (0-");
  Serial.print(FUTABA_BAUD_COUNT - 1);
  Serial.println("): ");

  String baudInput = readInput();
  int baudIndex = baudInput.toInt();

  // 範囲チェック
  if (!isNumber(baudInput, false) || baudIndex >= (int)FUTABA_BAUD_COUNT)
  {
    Serial.println("Error: Baud rate number out of range");
    return;
  }

  if (baudIndex > OFFICIAL_BAUD_MAX)
  {
    Serial.println("[Warning] This baud rate is outside the manufacturer's specification.");
  }

  Serial.println("Apply to: [1] ESP32 only");
  Serial.print("          [2] Servo ID ");
  Serial.print(targetServoID);
  Serial.println(" (flash ROM + reboot) + ESP32");
  Serial.println("          [3] ALL servos at every baud rate (MADI write) + ESP32");

  String modeInput = readInput();

  if (modeInput == "1")
  {
    applyHostBaudRate(baudIndex);
    return;
  }
  else if (modeInput == "3")
  {
    madiWriteBaudRate(baudIndex);
    return;
  }
  else if (modeInput != "2")
  {
    Serial.println("Error: Invalid selection");
    return;
  }

  writeServoBaudRate(baudIndex);
}

// サーボIDを oldId → newId に変更（書き込み → フラッシュ保存 → 読み返し確認）。
// 成功したら一斉制御リスト内の旧IDも置き換え、ターゲットを新IDにする
bool changeServoId(u8 oldId, u8 newId)
{
  if (newId == oldId)
  {
    Serial.println("New ID is the same as current ID. Nothing to do.");
    return true;
  }

  // ID重複チェック
  if (rsPing(newId, NULL, NULL, SCAN_TIMEOUT) != RS_NO_RESPONSE)
  {
    Serial.print("Error: ID ");
    Serial.print(newId);
    Serial.println(" is already in use");
    return false;
  }

  Serial.print("Changing servo ID ");
  Serial.print(oldId);
  Serial.print(" -> ");
  Serial.println(newId);

  // ID書き込み（即時反映）→ 新IDあてにフラッシュ保存 → 読み返し確認
  rsWriteValue(oldId, ADDR_SERVO_ID, 1, newId);
  delay(50);
  if (!saveAndVerify(newId, ADDR_SERVO_ID, newId))
  {
    Serial.println("Hint: Use [f] to scan servos");
    return false;
  }

  // 一斉制御リスト内の旧IDも新IDに置き換え（昇順を維持）
  for (u8 i = 0; i < syncServoCount; i++)
  {
    if (syncServoIDs[i] == oldId)
    {
      syncServoIDs[i] = newId;
      for (u8 j = i; j > 0 && syncServoIDs[j - 1] > syncServoIDs[j]; j--)
      {
        u8 tmp = syncServoIDs[j - 1];
        syncServoIDs[j - 1] = syncServoIDs[j];
        syncServoIDs[j] = tmp;
      }
      for (u8 j = i; j + 1 < syncServoCount && syncServoIDs[j] > syncServoIDs[j + 1]; j++)
      {
        u8 tmp = syncServoIDs[j + 1];
        syncServoIDs[j + 1] = syncServoIDs[j];
        syncServoIDs[j] = tmp;
      }
      break;
    }
  }

  targetServoID = newId;
  Serial.print("Servo ID changed successfully. Target ID is now ");
  Serial.println(targetServoID);
  return true;
}

// [i]コマンド: サーボID変更
void executeChangeServoIdCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);

  // 変更元サーボの存在確認
  if (rsPing(targetServoID, NULL, NULL, RESPONSE_TIMEOUT) != RS_OK)
  {
    Serial.println("Error: No response from target servo (use [z] to select ID)");
    return;
  }

  Serial.println("Enter new servo ID (1-127): ");

  String input = readInput();
  int newId = input.toInt();

  // ID範囲チェック
  if (!isNumber(input, false) || newId < FUTABA_ID_MIN || newId > FUTABA_ID_MAX)
  {
    Serial.println("Error: ID out of range (1-127)");
    return;
  }

  changeServoId(targetServoID, newId);
}

// [m]コマンド: サーボのバージョン情報と設定値の読み取り
void executeReadVersionCommand()
{
  Serial.print("Reading version info from servo ID ");
  Serial.println(targetServoID);

  // アドレス0から8バイト（型番2, FW版1, 予約1, ID, 反転, ボーレート, 返信ディレイ）
  u8 data[8];
  RsResult result = rsRead(targetServoID, ADDR_MODEL_NUMBER, 8, data);
  if (result != RS_OK)
  {
    Serial.print("Error: ");
    Serial.println(rsResultText(result));
    return;
  }

  u16 modelNumber = data[0] | (data[1] << 8);
  u8 baud = data[ADDR_BAUD_RATE];

  Serial.println("Protocol        : Futaba RS30x");
  Serial.print("Model number    : 0x");
  Serial.print(modelNumber, HEX);
  Serial.print(" -> ");
  Serial.println(modelName(modelNumber));
  Serial.print("Firmware version: ");
  Serial.println(data[ADDR_FIRMWARE_VERSION]);
  Serial.print("Servo ID        : ");
  Serial.println(data[ADDR_SERVO_ID]);
  Serial.print("Direction       : ");
  Serial.println(data[ADDR_REVERSE] ? "Reversed (CCW is +)" : "Normal (CW is +)");
  Serial.print("Baud rate (ROM) : ");
  Serial.print(baud);
  Serial.print(" (");
  Serial.print(baud < FUTABA_BAUD_COUNT ? FUTABA_BAUD_RATES[baud] : FUTABA_BAUD_RATES[DEFAULT_BAUD_INDEX]);
  Serial.println(" bps)");
  Serial.print("Return delay    : ");
  Serial.print(data[ADDR_RETURN_DELAY]);
  Serial.print(" (");
  Serial.print(100 + data[ADDR_RETURN_DELAY] * 50);
  Serial.println(" us)");
  Serial.print("Status flags    : ");
  printStatusFlags(lastStatusFlags);
  Serial.println();
}

// [o]コマンド: 回転方向設定
void executeSetDirectionCommand()
{
  long current;
  RsResult result = rsReadValue(targetServoID, ADDR_REVERSE, 1, &current, false);
  if (result != RS_OK)
  {
    Serial.print("Error: ");
    Serial.println(rsResultText(result));
    return;
  }

  Serial.print("Servo ID ");
  Serial.print(targetServoID);
  Serial.print(" direction: ");
  Serial.println(current ? "Reversed (CCW is +)" : "Normal (CW is +)");
  Serial.println("Select direction: [0] Normal (CW is +)  [1] Reversed (CCW is +)");

  String input = readInput();
  if (input != "0" && input != "1")
  {
    Serial.println("Error: Invalid selection");
    return;
  }

  u8 value = input.toInt();
  rsWriteValue(targetServoID, ADDR_REVERSE, 1, value);
  delay(10);
  saveAndVerify(targetServoID, ADDR_REVERSE, value);
}

// [l]コマンド: 返信ディレイ設定
void executeSetReturnDelayCommand()
{
  long current;
  RsResult result = rsReadValue(targetServoID, ADDR_RETURN_DELAY, 1, &current, false);
  if (result != RS_OK)
  {
    Serial.print("Error: ");
    Serial.println(rsResultText(result));
    return;
  }

  Serial.print("Servo ID ");
  Serial.print(targetServoID);
  Serial.print(" return delay: ");
  Serial.print(current);
  Serial.print(" (");
  Serial.print(100 + current * 50);
  Serial.println(" us)");
  Serial.println("Enter return delay (0-127, delay = 100us + value x 50us, 18 = 1ms): ");

  String input = readInput();
  int value = input.toInt();
  if (!isNumber(input, false) || value < 0 || value > 127)
  {
    Serial.println("Error: Value out of range (0-127)");
    return;
  }

  rsWriteValue(targetServoID, ADDR_RETURN_DELAY, 1, value);
  delay(10);
  if (saveAndVerify(targetServoID, ADDR_RETURN_DELAY, value))
  {
    Serial.print("Return delay set to ");
    Serial.print(100 + value * 50);
    Serial.println(" us");
  }
}

// [q]コマンド: 工場出荷リセット（全サーボ対象）
void executeFactoryResetCommand()
{
  Serial.println("[Warning] Factory reset is sent to ALL connected servos (broadcast).");
  Serial.println("          ID becomes 1 and baud rate becomes 115200 bps.");
  Serial.println("          Connect only ONE servo. Type YES to continue: ");

  if (readInput() != "YES")
  {
    Serial.println("Canceled");
    return;
  }

  Serial.println("Executing factory reset...");
  rsFactoryReset(BROADCAST_ID);
  rsFlashWrite(BROADCAST_ID);
  rsReboot(BROADCAST_ID);

  // 工場出荷値（ID1, 115200bps）に合わせる
  setHostBaudRate(DEFAULT_BAUD_INDEX);
  targetServoID = 1;

  u16 modelNumber;
  if (rsPing(targetServoID, &modelNumber, NULL, RESPONSE_TIMEOUT) == RS_OK)
  {
    Serial.print("Factory reset completed [OK]. Found ID 1 (");
    Serial.print(modelName(modelNumber));
    Serial.println(") at 115200 bps");
  }
  else
  {
    Serial.println("Verification failed: No response at ID 1 / 115200 bps");
    Serial.println("Hint: Use [f] to scan servos");
  }
}

// [t]コマンド: 送信テスト（ブロードキャストで全IDにトルクONと位置指令を送る。受信不要でTX側の確認用）
void executeTxTestCommand()
{
  Serial.println("TX test: broadcasting torque ON and position to ALL IDs: +30, -30, +30, -30, center.");
  Serial.println("No reception is needed. If the servo moves, TX (ESP32 -> servo) is OK.");

  rsWriteValue(BROADCAST_ID, ADDR_TORQUE_ENABLE, 1, 1);
  delay(10);

  const float angles[] = {30.0, -30.0, 30.0, -30.0, 0.0};
  for (u8 step = 0; step < sizeof(angles) / sizeof(angles[0]); step++)
  {
    // Goal Position(30, 0.1度) + Goal Time(32, 10ms)
    s16 position = (s16)(angles[step] * 10);
    const u16 goalTime = 50; // 0.5秒
    u8 data[4] = {(u8)(position & 0xFF), (u8)((position >> 8) & 0xFF), (u8)(goalTime & 0xFF), (u8)(goalTime >> 8)};
    rsWrite(BROADCAST_ID, ADDR_GOAL_POSITION, data, sizeof(data));

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
  Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps, Protocol: Futaba RS30x");
  Serial.println("Menu: [f] Find servos   [n] Set sync servo IDs   [z] Set target ID ");
  Serial.println("      [v] Read voltage  [r] Read position        [p] Read register");
  Serial.println("      [c] All to center [a] Read all positions   [s] Write register");
  Serial.println("      [e] All torque ON [d] All torque OFF       [x] Send angle to all ");
  Serial.println("      [b] Set baud rate [i] Change servo ID      [m] Read version");
  Serial.println("      [o] Set direction [l] Set return delay     [q] Factory reset");
  Serial.println("      [t] TX test (all IDs) [h] Change servo type (restart)");
  Serial.println("Input Command:");
}

// 初期化
void setup()
{
  Serial.begin(115200);
  // サーボバス開始（8N1, エコー自動判定, 送信モードで待機）
  busBegin(&Serial1, RX1, TX1, PIN_EN, FUTABA_BAUD_RATES[servoBaudIndex], SERIAL_8N1);

  delay(1000);
  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());

  Serial.println("=====================================");
  Serial.println("Futaba TTL Servo Setter");
  Serial.println("RS30x series (RS303MR / RS304MD)");
  Serial.println("=====================================");
  Serial.println("Use [f] to find servos first.");

  disp_menu();
}

void loop()
{
  // シリアル入力チェック
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n'); // 改行を受けたらすぐ処理（ブラウザからの操作を速くする）
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
      setTorqueAll(1);
    }
    else if (command == "d")
    {
      setTorqueAll(0);
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
    else if (command == "o")
    {
      executeSetDirectionCommand();
    }
    else if (command == "l")
    {
      executeSetReturnDelayCommand();
    }
    else if (command == "q")
    {
      executeFactoryResetCommand();
    }
    else if (command == "t")
    {
      executeTxTestCommand();
    }
    else if (command.startsWith("@"))
    {
      handleApiCommand(command); // ブラウザ用の機械向けコマンド
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

// ===== ブラウザ用API =====

// 今の状態と、この種類でできること
void apiGetInfo(ApiInfo *info)
{
  info->idMin = FUTABA_ID_MIN;
  info->idMax = FUTABA_ID_MAX;
  info->angleMin = -ANGLE_LIMIT;
  info->angleMax = ANGLE_LIMIT;
  info->bauds = FUTABA_BAUD_RATES;
  info->baudCount = FUTABA_BAUD_COUNT;
  info->baudIndex = servoBaudIndex;
  info->baud = FUTABA_BAUD_RATES[servoBaudIndex];
  info->target = targetServoID;
  info->syncIds = syncServoIDs;
  info->syncCount = syncServoCount;
  info->oneServoForId = false;
}

// [f] と同じ検索（今の通信速度だけ。全速度の検索はしない）。見つかった数を返す
int apiScan(int maxId, ApiServo *found, int maxCount)
{
  static u8 foundServos[MAX_SYNC_SERVOS];
  static u16 foundModels[MAX_SYNC_SERVOS];

  // ID範囲チェック
  if (maxId < FUTABA_ID_MIN || maxId > FUTABA_ID_MAX)
  {
    Serial.println("Error: ID out of range (1-127)");
    return 0;
  }

  u8 foundCount = scanAndApply(maxId, foundServos, foundModels);
  if (foundCount == 0)
  {
    Serial.println();
    Serial.print("No servos detected at ");
    Serial.print(FUTABA_BAUD_RATES[servoBaudIndex]);
    Serial.println(" bps");
  }

  int count = 0;
  for (u8 i = 0; i < foundCount && count < maxCount; i++)
  {
    found[count].id = foundServos[i];
    strncpy(found[count].model, modelName(foundModels[i]), API_MODEL_LEN - 1);
    found[count].model[API_MODEL_LEN - 1] = '\0';
    count++;
  }
  return count;
}

// ターゲットID変更（[z]）
bool apiSetTarget(int id)
{
  if (id < FUTABA_ID_MIN || id > FUTABA_ID_MAX)
    return false;
  targetServoID = id;
  return true;
}

// 一斉制御リスト設定（[n]）。昇順・重複なしにして格納
bool apiSetSync(const uint8_t *ids, uint8_t count)
{
  bool selected[FUTABA_ID_MAX + 1] = {false};
  if (count == 0)
    return false;
  for (u8 i = 0; i < count; i++)
  {
    if (ids[i] < FUTABA_ID_MIN || ids[i] > FUTABA_ID_MAX)
      return false;
    selected[ids[i]] = true;
  }

  u8 newCount = 0;
  for (int id = FUTABA_ID_MIN; id <= FUTABA_ID_MAX; id++)
  {
    if (selected[id])
      syncServoIDs[newCount++] = id;
  }
  syncServoCount = newCount;
  return true;
}

// 一斉制御リストを角度へ（[x]）
bool apiMove(float deg)
{
  // 角度範囲チェック
  if (deg < -ANGLE_LIMIT || deg > ANGLE_LIMIT)
    return false;

  s16 position = (s16)(deg * 10.0 + (deg >= 0 ? 0.5 : -0.5)); // 0.1度単位に四捨五入
  Serial.print("Converting ");
  Serial.print(deg);
  Serial.print(" degrees to position ");
  Serial.println(position);

  moveSyncServos(position, MOVE_GOAL_TIME);
  Serial.println("Angle command sent successfully");
  return true;
}

// 一斉制御リストを中央へ（[c]）
bool apiCenter()
{
  Serial.print("Resetting ");
  Serial.print(syncServoCount);
  Serial.println(" servos to position 0 (center)...");

  moveSyncServos(0, CENTER_GOAL_TIME);
  Serial.println("All servos reset to center position");
  return true;
}

// トルクON/OFF（[e]/[d]）。全台成功で true
bool apiTorque(bool on)
{
  return setTorqueAll(on ? 1 : 0);
}

// 送信テスト（[t]）
bool apiTxTest()
{
  executeTxTestCommand();
  return true;
}

// 現在位置の読み取り（[r]）。度で返す
bool apiReadPosition(int id, float *deg)
{
  if (id < FUTABA_ID_MIN || id > FUTABA_ID_MAX)
    return false;

  Serial.print("Reading position from servo ID ");
  Serial.println(id);

  long position;
  RsResult result = rsReadValue(id, ADDR_PRESENT_POSITION, 2, &position, true);
  if (result != RS_OK)
  {
    Serial.print("Error: ");
    Serial.println(rsResultText(result));
    return false;
  }

  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" (");
  Serial.print(position / 10.0, 1);
  Serial.print("°)");
  Serial.println(position == POSITION_DEAD_ZONE ? " [Dead zone: shaft is out of range]" : "");

  *deg = position / 10.0;
  return true;
}

// サーボID変更（[i]）
bool apiSetId(int oldId, int newId)
{
  if (oldId < FUTABA_ID_MIN || oldId > FUTABA_ID_MAX)
    return false;

  // 変更元サーボの存在確認
  if (rsPing(oldId, NULL, NULL, RESPONSE_TIMEOUT) != RS_OK)
  {
    Serial.println("Error: No response from target servo (use [z] to select ID)");
    return false;
  }

  // ID範囲チェック
  if (newId < FUTABA_ID_MIN || newId > FUTABA_ID_MAX)
  {
    Serial.println("Error: ID out of range (1-127)");
    return false;
  }

  return changeServoId(oldId, newId);
}

// 通信速度変更（[b] の [1] ESP32だけ / [2] ターゲットサーボ + ESP32）
bool apiSetBaud(int index, bool servoToo)
{
  // 範囲チェック
  if (index < 0 || index >= (int)FUTABA_BAUD_COUNT)
    return false;

  if (index > OFFICIAL_BAUD_MAX)
  {
    Serial.println("[Warning] This baud rate is outside the manufacturer's specification.");
  }

  if (!servoToo)
  {
    applyHostBaudRate(index);
    return true;
  }
  return writeServoBaudRate(index);
}

// ===== サーボ種類の自動判別（起動時のメニュー [0]） =====

// 読み取りコマンドだけでRS30x のサーボを探す（サーボは動かない）。
// bauds: 調べる通信速度（速い順に調べる）。見つかった通信速度（なければ0）
unsigned long detect(DetectBauds bauds)
{
  const u8 original = servoBaudIndex;
  unsigned long found = 0;
  busBegin(servoSerial, RX1, TX1, PIN_EN, FUTABA_BAUD_RATES[DEFAULT_BAUD_INDEX], SERIAL_8N1);
  for (int b = FUTABA_BAUD_COUNT - 1; b >= 0 && found == 0; b--)
  {
    if (detectSkipsBaud(bauds, b == DEFAULT_BAUD_INDEX, FUTABA_BAUD_RATES[b]))
      continue;
    setHostBaudRate(b);
    for (int id = FUTABA_ID_MIN; id <= DEFAULT_SCAN_MAX_ID; id++)
    {
      if (rsPing(id, NULL, NULL, detectTimeout(FUTABA_BAUD_RATES[b], 12)) == RS_OK)
      {
        found = FUTABA_BAUD_RATES[b];
        break;
      }
    }
  }
  servoBaudIndex = original;
  return found;
}

// 起動前に通信速度を設定する（自動判別で見つかった速度で始めるため）
void presetBaud(unsigned long baud)
{
  for (u8 i = 0; i < FUTABA_BAUD_COUNT; i++)
  {
    if (FUTABA_BAUD_RATES[i] == baud)
      servoBaudIndex = i;
  }
}

} // namespace futaba
