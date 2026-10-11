#include <Arduino.h>
#include "servo_bus.h"
#include "servo_select.h"

namespace dynamixel
{

// 基本型定義
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;
typedef long s32;

// ピン定義
#define PIN_EN 33 // 半二重通信の送受信切替（HIGH: 送信, LOW: 受信）

// Dynamixel Protocol 2.0 命令定義
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_SYNC_WRITE 0x83
#define INST_STATUS 0x55 // ステータスパケットの命令値

// ID定義
#define BROADCAST_ID 0xFE
#define DXL_ID_MAX 252

// パケット構造定義（FF FF FD 00 ID LEN_L LEN_H INST ...）
#define PKT_HEADER_SIZE 7 // ヘッダ4 + ID1 + 長さ2
#define PKT_INSTRUCTION 7 // 命令バイトの位置
#define PKT_ERROR 8       // ステータスパケットのエラーバイト位置
#define PKT_STATUS_PARAM 9 // ステータスパケットのパラメータ開始位置
#define MAX_PACKET_SIZE 1024
#define MAX_WRITE_DATA 16

// コントロールテーブル定義（Xシリーズ / MX(2.0)シリーズ共通）
#define ADDR_MODEL_NUMBER 0            // 2バイト
#define ADDR_MODEL_INFORMATION 2       // 4バイト
#define ADDR_FIRMWARE_VERSION 6        // 1バイト
#define ADDR_ID 7                      // 1バイト（EEPROM）
#define ADDR_BAUD_RATE 8               // 1バイト（EEPROM）
#define ADDR_MAX_VOLTAGE_LIMIT 32      // 2バイト（EEPROM, 0.1V）
#define ADDR_MIN_VOLTAGE_LIMIT 34      // 2バイト（EEPROM, 0.1V）
#define ADDR_TORQUE_ENABLE 64          // 1バイト
#define ADDR_HARDWARE_ERROR_STATUS 70  // 1バイト
#define ADDR_PROFILE_ACCELERATION 108  // 4バイト（108-119: 加速度 + 速度 + 目標位置）
#define ADDR_PRESENT_POSITION 132      // 4バイト
#define ADDR_PRESENT_INPUT_VOLTAGE 144 // 2バイト（0.1V）
#define ADDR_EEPROM_END 64             // これ未満はEEPROM領域（トルクOFF時のみ書き込み可）

// 位置値の定数定義（位置制御モード）
#define POSITION_MIN 0
#define POSITION_MAX 4095
#define POSITION_CENTER 2048
#define ANGLE_RANGE 360.0

// 一斉移動のプロファイル（Drive Mode = 速度ベース時の単位）
#define MOVE_PROFILE_ACCELERATION 20 // 214.577 rev/min^2 単位
#define MOVE_PROFILE_VELOCITY 100    // 0.229 rev/min 単位（約23rpm）
#define CENTER_PROFILE_VELOCITY 60   // 中央復帰は少し遅め（約14rpm）

// 一斉制御定義
#define MAX_SYNC_SERVOS 253      // 一斉制御リストの最大数（有効ID 0-252）
#define SYNC_WRITE_MAX_SERVOS 30 // 1パケットあたりの最大台数（超える場合は分割送信）

// 通信タイムアウト定義
#define RESPONSE_TIMEOUT 100 // 通常の応答待ち(ms)
#define SCAN_TIMEOUT 30      // サーボ検索時の応答待ち(ms)

// サーボ検索のデフォルト最大ID（[f]で空欄リターン時）
#define DEFAULT_SCAN_MAX_ID 20

// Dynamixel標準通信速度（インデックス = ボーレートレジスタ値）
const unsigned long DXL_BAUD_RATES[] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000};
#define DXL_BAUD_COUNT (sizeof(DXL_BAUD_RATES) / sizeof(DXL_BAUD_RATES[0]))
#define DEFAULT_BAUD_INDEX 1 // 57600bps（Xシリーズ工場出荷値）

// 既知のサーボ型番（ROBOTIS Dynamixel2Arduinoの型番定義より）
struct DxlModel
{
  u16 number;
  const char *name;
  bool supported; // このツールのコントロールテーブル（X/MX(2.0)共通）に対応しているか
};
const DxlModel DXL_MODELS[] = {
    {1060, "XL430-W250", true},
    {1090, "2XL430-W250", true},
    {1160, "2XC430-W250", true},
    {1070, "XC430-W150", true},
    {1080, "XC430-W240", true},
    {1190, "XL330-M077", true},
    {1200, "XL330-M288", true},
    {1210, "XC330-T181", true},
    {1220, "XC330-T288", true},
    {1230, "XC330-M181", true},
    {1240, "XC330-M288", true},
    {1030, "XM430-W210", true},
    {1020, "XM430-W350", true},
    {1130, "XM540-W150", true},
    {1120, "XM540-W270", true},
    {1010, "XH430-W210", true},
    {1000, "XH430-W350", true},
    {1050, "XH430-V210", true},
    {1040, "XH430-V350", true},
    {1110, "XH540-W150", true},
    {1100, "XH540-W270", true},
    {1150, "XH540-V150", true},
    {1140, "XH540-V270", true},
    {1011, "XD430-T210", true},
    {1001, "XD430-T350", true},
    {1111, "XD540-T150", true},
    {1101, "XD540-T270", true},
    {1280, "XW430-T200", true},
    {1270, "XW430-T333", true},
    {1180, "XW540-T140", true},
    {1170, "XW540-T260", true},
    {30, "MX-28(2.0)", true},
    {311, "MX-64(2.0)", true},
    {321, "MX-106(2.0)", true},
    {350, "XL-320", false}, // コントロールテーブルが異なるため非対応
};
#define DXL_MODEL_COUNT (sizeof(DXL_MODELS) / sizeof(DXL_MODELS[0]))

// ステータスパケットのエラー番号名（0x01-0x07）
const char *DXL_ERROR_NAMES[] = {
    "No Error", "Result Fail", "Instruction Error", "CRC Error",
    "Data Range Error", "Data Length Error", "Data Limit Error", "Access Error"};

// 通信結果
enum DxlResult
{
  DXL_OK,
  DXL_NO_RESPONSE,
  DXL_INVALID_RESPONSE,
  DXL_STATUS_ERROR,
  DXL_PACKET_ERROR
};

// グローバル変数
HardwareSerial *servoSerial = &Serial1;                 // 使用するシリアルポート
u8 targetServoID = 1;                                   // ターゲットサーボID
u8 syncServoIDs[MAX_SYNC_SERVOS] = {1, 2, 3, 4, 5, 6}; // 一斉制御するサーボIDリスト（とびとびIDも可）
u8 syncServoCount = 6;                                  // リスト内のサーボ数
u8 servoBaudIndex = DEFAULT_BAUD_INDEX;                 // ESP32側の現在の通信速度インデックス
u8 lastStatusError = 0;                                 // 最後に受信したステータスパケットのエラーバイト

// 送受信バッファ
u8 txPacket[MAX_PACKET_SIZE];
u8 rxPacket[MAX_PACKET_SIZE];

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

// 4バイト値をリトルエンディアンで格納
void putU32(u8 *data, u32 value)
{
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
  data[2] = (value >> 16) & 0xFF;
  data[3] = (value >> 24) & 0xFF;
}

// リトルエンディアンの値を取り出し（1/2/4バイト）
u32 getValue(const u8 *data, u8 size)
{
  u32 value = 0;
  for (u8 i = 0; i < size; i++)
  {
    value |= (u32)data[i] << (8 * i);
  }
  return value;
}

// 位置値から角度への変換（中央 = 0度）
float positionToAngle(s32 position)
{
  return (float)(position - POSITION_CENTER) / (POSITION_MAX + 1) * ANGLE_RANGE;
}

// 角度から位置値への変換（-180°〜180° → 0-4095）
s32 angleToPosition(float angle)
{
  s32 position = POSITION_CENTER + (s32)(angle / ANGLE_RANGE * (POSITION_MAX + 1));
  if (position < POSITION_MIN)
    position = POSITION_MIN;
  if (position > POSITION_MAX)
    position = POSITION_MAX;
  return position;
}

// 型番検索（見つからなければNULL）
const DxlModel *findModel(u16 modelNumber)
{
  for (u8 i = 0; i < DXL_MODEL_COUNT; i++)
  {
    if (DXL_MODELS[i].number == modelNumber)
      return &DXL_MODELS[i];
  }
  return NULL;
}

// 型番名（未知の型番は"Unknown"）
const char *modelName(u16 modelNumber)
{
  const DxlModel *model = findModel(modelNumber);
  return model ? model->name : "Unknown";
}

// このツールで扱える型番か（未知の型番はX系とみなして許可）
bool isSupportedModel(u16 modelNumber)
{
  const DxlModel *model = findModel(modelNumber);
  return model ? model->supported : true;
}

// 通信結果の表示用テキスト
const char *dxlResultText(DxlResult result)
{
  switch (result)
  {
  case DXL_OK:
    return "OK";
  case DXL_NO_RESPONSE:
    return "No response";
  case DXL_INVALID_RESPONSE:
    return "Invalid response (format/CRC)";
  case DXL_STATUS_ERROR:
    return ((lastStatusError & 0x7F) <= 0x07) ? DXL_ERROR_NAMES[lastStatusError & 0x7F] : "Unknown status error";
  default:
    return "Packet build error";
  }
}

// ===== Protocol 2.0 通信レイヤー =====

// CRC-16計算（多項式0x8005, 初期値0。ROBOTIS e-Manualのupdate_crcと同等）
u16 updateCRC(u16 crc, const u8 *data, u16 size)
{
  for (u16 i = 0; i < size; i++)
  {
    crc ^= (u16)data[i] << 8;
    for (u8 bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
    }
  }
  return crc;
}

// 命令パケット構築（バイトスタッフィング・CRC付き）。戻り値はパケット長（0はエラー）
u16 buildPacket(u8 *packet, u8 id, u8 instruction, const u8 *params, u16 paramLength)
{
  u16 index = 0;

  // ヘッダ構築
  packet[index++] = 0xFF;
  packet[index++] = 0xFF;
  packet[index++] = 0xFD;
  packet[index++] = 0x00; // 予約
  packet[index++] = id;
  packet[index++] = 0; // 長さ（後で設定）
  packet[index++] = 0;
  packet[index++] = instruction;

  // パラメータ追加（FF FF FD が現れたら FD を挿入）
  for (u16 i = 0; i < paramLength; i++)
  {
    if (index + 4 > MAX_PACKET_SIZE) // スタッフィング1 + CRC2 の余裕
      return 0;

    packet[index++] = params[i];
    if (index >= PKT_INSTRUCTION + 3 &&
        packet[index - 3] == 0xFF && packet[index - 2] == 0xFF && packet[index - 1] == 0xFD)
    {
      packet[index++] = 0xFD;
    }
  }

  // 長さ = 命令 + パラメータ（スタッフィング後） + CRC
  u16 length = index - PKT_INSTRUCTION + 2;
  packet[5] = length & 0xFF;
  packet[6] = length >> 8;

  // CRC追加（ヘッダからパラメータまで）
  u16 crc = updateCRC(0, packet, index);
  packet[index++] = crc & 0xFF;
  packet[index++] = crc >> 8;

  return index;
}

// 受信バッファフラッシュ
void flushReceiveBuffer()
{
  while (servoSerial->read() != -1)
    ;
}

// 半二重通信でパケット送信
// toRx = true : 送信完了後すぐ受信モードに切り替え、送信エコーを読み捨てる
// toRx = false: 送信後も送信モードのまま（ブロードキャストなど応答なし）
void sendPacket(const u8 *packet, u16 length, bool toRx)
{
  // 受信バッファクリア → 送信
  flushReceiveBuffer();
  busDirTx();
  servoSerial->write(packet, length);

  if (toRx)
  {
    busEndSendToRx();
    busSkipEcho(packet, length, 10);
  }
  else
  {
    servoSerial->flush();
  }
}

// ステータスパケット受信（ヘッダ同期・CRC検証・スタッフィング除去）
DxlResult receiveStatus(u8 id, u8 *params, u16 maxParams, u16 *paramLength, unsigned long timeout)
{
  const u8 header[4] = {0xFF, 0xFF, 0xFD, 0x00};
  u16 index = 0;
  u16 expectedLength = 0; // パケット全体長（ヘッダ受信後に確定）
  unsigned long startTime = millis();

  *paramLength = 0;

  while ((millis() - startTime) < timeout)
  {
    if (!servoSerial->available())
      continue;

    u8 data = servoSerial->read();

    // ヘッダ（FF FF FD 00）同期
    if (index < 4)
    {
      if (data == header[index])
        index++;
      else if (data == 0xFF)
        index = (index == 2) ? 2 : 1; // FF FF FF... の場合も同期を維持
      else
        index = 0;
      continue;
    }

    rxPacket[index++] = data;

    // 長さフィールド受信でパケット全体長を確定
    if (index == PKT_HEADER_SIZE)
    {
      u16 length = rxPacket[5] | (rxPacket[6] << 8);
      expectedLength = PKT_HEADER_SIZE + length;
      if (length < 4 || expectedLength > MAX_PACKET_SIZE)
        return DXL_INVALID_RESPONSE;
    }

    if (expectedLength > 0 && index >= expectedLength)
      break;
  }

  if (expectedLength == 0 || index < expectedLength)
    return (index == 0) ? DXL_NO_RESPONSE : DXL_INVALID_RESPONSE;

  memcpy(rxPacket, header, 4);

  // CRC検証
  u16 crc = updateCRC(0, rxPacket, expectedLength - 2);
  u16 receivedCrc = rxPacket[expectedLength - 2] | (rxPacket[expectedLength - 1] << 8);
  if (crc != receivedCrc)
    return DXL_INVALID_RESPONSE;

  // ID・命令検証
  if (rxPacket[4] != id || rxPacket[PKT_INSTRUCTION] != INST_STATUS)
    return DXL_INVALID_RESPONSE;

  lastStatusError = rxPacket[PKT_ERROR];

  // パラメータ取り出し（FF FF FD FD の最後のFDを除去）
  u16 count = 0;
  for (u16 i = PKT_STATUS_PARAM; i < expectedLength - 2; i++)
  {
    if (rxPacket[i] == 0xFD && rxPacket[i - 1] == 0xFD && rxPacket[i - 2] == 0xFF && rxPacket[i - 3] == 0xFF)
      continue;

    if (count >= maxParams)
      return DXL_INVALID_RESPONSE;
    params[count++] = rxPacket[i];
  }
  *paramLength = count;

  // エラー番号（下位7ビット）があれば失敗。アラートビット（bit7）は処理成功扱い
  return (lastStatusError & 0x7F) ? DXL_STATUS_ERROR : DXL_OK;
}

// 命令送信→ステータス受信（ブロードキャストは応答なし）
DxlResult dxlTransact(u8 id, u8 instruction, const u8 *params, u16 paramLength,
                      u8 *rxParams, u16 maxRxParams, u16 *rxLength, unsigned long timeout)
{
  u16 length = buildPacket(txPacket, id, instruction, params, paramLength);
  if (length == 0)
    return DXL_PACKET_ERROR;

  // ブロードキャストは応答なし（送信モードを維持）
  if (id == BROADCAST_ID)
  {
    sendPacket(txPacket, length, false);
    delayMicroseconds(50);
    return DXL_OK;
  }

  // 送信 → すぐ受信モードに切り替え（エコーは読み捨て済み）
  sendPacket(txPacket, length, true);

  u16 dummyLength;
  DxlResult result = receiveStatus(id, rxParams, maxRxParams, rxLength ? rxLength : &dummyLength, timeout);

  // 送信モードに復帰
  busDirTx();

  // アラート（ハードウェアエラー）通知
  if (result != DXL_NO_RESPONSE && result != DXL_INVALID_RESPONSE && (lastStatusError & 0x80))
  {
    Serial.print("[Alert] ID ");
    Serial.print(id);
    Serial.println(": Hardware error detected (see [m] for details)");
  }

  return result;
}

// PING（応答で型番とファームウェア版を取得）
DxlResult dxlPing(u8 id, u16 *modelNumber, u8 *firmwareVersion, unsigned long timeout)
{
  u8 response[3];
  u16 length;
  DxlResult result = dxlTransact(id, INST_PING, NULL, 0, response, sizeof(response), &length, timeout);
  if (result == DXL_OK && length != 3)
    result = DXL_INVALID_RESPONSE;

  if (result == DXL_OK)
  {
    if (modelNumber)
      *modelNumber = response[0] | (response[1] << 8);
    if (firmwareVersion)
      *firmwareVersion = response[2];
  }
  return result;
}

// READ（指定アドレスからlengthバイト読み取り）
DxlResult dxlRead(u8 id, u16 address, u16 length, u8 *data, unsigned long timeout = RESPONSE_TIMEOUT)
{
  u8 params[4] = {(u8)(address & 0xFF), (u8)(address >> 8), (u8)(length & 0xFF), (u8)(length >> 8)};
  u16 received;
  DxlResult result = dxlTransact(id, INST_READ, params, 4, data, length, &received, timeout);
  if (result == DXL_OK && received != length)
    result = DXL_INVALID_RESPONSE;
  return result;
}

// WRITE（指定アドレスにlengthバイト書き込み、ステータスで結果確認）
DxlResult dxlWrite(u8 id, u16 address, const u8 *data, u8 length)
{
  if (length > MAX_WRITE_DATA)
    return DXL_PACKET_ERROR;

  u8 params[2 + MAX_WRITE_DATA];
  params[0] = address & 0xFF;
  params[1] = address >> 8;
  memcpy(&params[2], data, length);

  return dxlTransact(id, INST_WRITE, params, 2 + length, NULL, 0, NULL, RESPONSE_TIMEOUT);
}

// 1/2/4バイト値の読み取り
DxlResult dxlReadValue(u8 id, u16 address, u8 size, u32 *value, unsigned long timeout = RESPONSE_TIMEOUT)
{
  u8 data[4] = {0, 0, 0, 0};
  DxlResult result = dxlRead(id, address, size, data, timeout);
  if (result == DXL_OK)
    *value = getValue(data, size);
  return result;
}

// 1/2/4バイト値の書き込み
DxlResult dxlWriteValue(u8 id, u16 address, u8 size, u32 value)
{
  u8 data[4];
  putU32(data, value);
  return dxlWrite(id, address, data, size);
}

// SYNC WRITE（同一アドレス・同一長を複数サーボへ。上限台数を超える場合は分割送信）
bool dxlSyncWrite(u16 address, u16 dataLength, const u8 *ids, u8 count, const u8 *data)
{
  static u8 params[MAX_PACKET_SIZE];

  for (u16 offset = 0; offset < count; offset += SYNC_WRITE_MAX_SERVOS)
  {
    u8 n = (count - offset < SYNC_WRITE_MAX_SERVOS) ? (count - offset) : SYNC_WRITE_MAX_SERVOS;
    u16 index = 0;

    params[index++] = address & 0xFF;
    params[index++] = address >> 8;
    params[index++] = dataLength & 0xFF;
    params[index++] = dataLength >> 8;

    for (u8 i = 0; i < n; i++)
    {
      params[index++] = ids[offset + i];
      memcpy(&params[index], &data[(offset + i) * dataLength], dataLength);
      index += dataLength;
    }

    if (dxlTransact(BROADCAST_ID, INST_SYNC_WRITE, params, index, NULL, 0, NULL, 0) != DXL_OK)
    {
      Serial.println("Error: SyncWrite packet too large");
      return false;
    }
  }
  return true;
}

// ESP32側のサーボ通信速度を変更
void setHostBaudRate(u8 baudIndex)
{
  servoSerial->flush();
  servoSerial->updateBaudRate(DXL_BAUD_RATES[baudIndex]);
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
  bool selected[DXL_ID_MAX + 1] = {false};
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
    if (first < 0 || last > DXL_ID_MAX || first > last)
      return -1;

    for (int id = first; id <= last; id++)
      selected[id] = true;
  }

  int count = 0;
  for (int id = 0; id <= DXL_ID_MAX; id++)
  {
    if (selected[id])
      out[count++] = id;
  }
  return count > 0 ? count : -1;
}

// 一斉制御リストの全サーボを指定位置へ移動（トルクON → プロファイル + 目標位置）
bool moveSyncServos(s32 position, u32 velocity)
{
  static u8 data[MAX_SYNC_SERVOS * 12];

  // Dynamixelは目標位置だけでは動かないため、先にトルクをONにする
  for (u8 i = 0; i < syncServoCount; i++)
    data[i] = 1;
  if (!dxlSyncWrite(ADDR_TORQUE_ENABLE, 1, syncServoIDs, syncServoCount, data))
    return false;

  // Profile Acceleration(108) + Profile Velocity(112) + Goal Position(116) を一括書き込み
  for (u8 i = 0; i < syncServoCount; i++)
  {
    putU32(&data[i * 12], MOVE_PROFILE_ACCELERATION);
    putU32(&data[i * 12 + 4], velocity);
    putU32(&data[i * 12 + 8], (u32)position);
  }

  Serial.print("SyncWrite: ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.print("), Pos=");
  Serial.print(positionToAngle(position), 1);
  Serial.print("deg (");
  Serial.print(position);
  Serial.print("), Velocity=");
  Serial.print(velocity);
  Serial.print(", Acc=");
  Serial.println(MOVE_PROFILE_ACCELERATION);

  return dxlSyncWrite(ADDR_PROFILE_ACCELERATION, 12, syncServoIDs, syncServoCount, data);
}

// ===== コマンド =====

// 指定範囲のIDをPINGで検索（現在のESP32通信速度）。見つかった数を返す
u8 scanServos(int maxId, u8 *foundServos, u16 *foundModels, bool showProgress)
{
  u8 foundCount = 0;

  // プログレス表示用
  int progressStep = (maxId + 1) / 10;
  if (progressStep == 0)
    progressStep = 1;

  for (int id = 0; id <= maxId; id++)
  {
    // プログレス表示
    if (showProgress && ((id + 1) % progressStep == 0 || id == maxId))
    {
      Serial.print("Progress: ");
      Serial.print(((id + 1) * 100) / (maxId + 1));
      Serial.println("%");
    }

    u16 modelNumber;
    u8 firmwareVersion;
    if (dxlPing(id, &modelNumber, &firmwareVersion, SCAN_TIMEOUT) == DXL_OK)
    {
      foundServos[foundCount] = id;
      foundModels[foundCount] = modelNumber;
      foundCount++;

      Serial.print("Found servo ID: ");
      Serial.print(id);
      Serial.print(" (");
      Serial.print(modelName(modelNumber));
      Serial.print(", model ");
      Serial.print(modelNumber);
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
  Serial.print("Scanned range: ID0 to ID");
  Serial.print(maxId);
  Serial.print(" at ");
  Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  Serial.print("Found ");
  Serial.print(foundCount);
  Serial.println(" servo(s)");

  Serial.print("Connected IDs: ");
  u8 supportedCount = 0;
  for (u8 i = 0; i < foundCount; i++)
  {
    Serial.print(foundServos[i]);
    Serial.print("(");
    Serial.print(modelName(foundModels[i]));
    Serial.print(")");
    if (i < foundCount - 1)
      Serial.print(", ");
    if (isSupportedModel(foundModels[i]))
      supportedCount++;
  }
  Serial.println();

  if (supportedCount < foundCount)
  {
    Serial.println("[Warning] Unsupported models (e.g. XL-320) are excluded from target/sync IDs.");
  }

  // 見つかったIDにターゲットとリストを自動設定（対応型番を優先、なければ全て）
  u8 count = 0;
  for (u8 i = 0; i < foundCount; i++)
  {
    if (supportedCount == 0 || isSupportedModel(foundModels[i]))
      syncServoIDs[count++] = foundServos[i];
  }
  syncServoCount = count;
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
  static u8 ids[DXL_ID_MAX + 1];
  static u16 models[DXL_ID_MAX + 1];
  static u8 bestIds[DXL_ID_MAX + 1];
  static u16 bestModels[DXL_ID_MAX + 1];

  u8 originalIndex = servoBaudIndex;
  int bestIndex = -1;
  u8 bestCount = 0;
  u8 baudsWithServos = 0;

  Serial.println();
  Serial.println("=== Scanning all baud rates (fastest first) ===");

  for (int b = DXL_BAUD_COUNT - 1; b >= 0; b--)
  {
    setHostBaudRate(b);
    Serial.print("Scanning at ");
    Serial.print(DXL_BAUD_RATES[b]);
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
    Serial.println("Hint: Check wiring/power, and that the servo uses Protocol 2.0");
    Serial.println();
    return;
  }

  setHostBaudRate(bestIndex);
  Serial.println();
  Serial.print("ESP32 baud rate auto-set to: ");
  Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");

  if (baudsWithServos > 1)
  {
    Serial.println("[Warning] Servos were found at multiple baud rates.");
    Serial.println("          Using the baud rate with the most servos. Use [b] to unify them.");
  }

  applyScanResult(maxId, bestIds, bestModels, bestCount);
}

// 現在の通信速度で検索し、見つかればターゲットIDと一斉制御リストを自動設定。見つかった数を返す
u8 scanAndApply(int maxId, u8 *foundServos, u16 *foundModels)
{
  Serial.print("Scanning servos from ID0 to ID");
  Serial.print(maxId);
  Serial.print(" at ");
  Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
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
  static u8 foundServos[DXL_ID_MAX + 1];
  static u16 foundModels[DXL_ID_MAX + 1];

  Serial.print("Enter maximum ID to scan (0-252, empty = ");
  Serial.print(DEFAULT_SCAN_MAX_ID);
  Serial.println("): ");

  String input = readInput();

  // 空欄リターンはデフォルト最大IDで検索
  int maxId = input.length() == 0 ? DEFAULT_SCAN_MAX_ID : input.toInt();

  // ID範囲チェック
  if (maxId < 0 || maxId > DXL_ID_MAX || (input.length() > 0 && !isDigit(input[0])))
  {
    Serial.println("Error: ID out of range (0-252)");
    return;
  }

  if (scanAndApply(maxId, foundServos, foundModels) > 0)
    return;

  Serial.println();
  Serial.print("No servos detected at ");
  Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
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
  Serial.println("Enter new target ID (0-252): ");

  String input = readInput();

  // 数字でなければエラー（範囲チェックと変更は apiSetTarget）
  if (input.length() == 0 || !isDigit(input[0]) || !apiSetTarget(input.toInt()))
  {
    Serial.println("Error: ID out of range (0-252)");
  }
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
    Serial.println("Error: Invalid ID list (valid IDs: 0-252)");
  }
}

// [r]コマンド: サーボ位置読み取り
void executeReadPositionCommand()
{
  float angle;
  apiReadPosition(targetServoID, &angle);
}

// [v]コマンド: サーボ電圧読み取り（電圧リミットと比較）
void executeReadVoltageCommand()
{
  Serial.print("Reading voltage from servo ID ");
  Serial.println(targetServoID);

  u32 voltage;
  DxlResult result = dxlReadValue(targetServoID, ADDR_PRESENT_INPUT_VOLTAGE, 2, &voltage);
  if (result != DXL_OK)
  {
    Serial.print("Error: ");
    Serial.println(dxlResultText(result));
    return;
  }

  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" (");
  Serial.print(voltage / 10.0, 1);
  Serial.print("V)");

  // サーボ自身の電圧リミット（Max: 32, Min: 34）と比較
  u8 limits[4];
  if (dxlRead(targetServoID, ADDR_MAX_VOLTAGE_LIMIT, 4, limits) == DXL_OK)
  {
    u16 maxLimit = getValue(&limits[0], 2);
    u16 minLimit = getValue(&limits[2], 2);
    if (voltage < minLimit)
      Serial.print(" [Warning] Low voltage");
    else if (voltage > maxLimit)
      Serial.print(" [Warning] High voltage");
    else
      Serial.print(" [OK]");

    Serial.print(" (limit ");
    Serial.print(minLimit / 10.0, 1);
    Serial.print("-");
    Serial.print(maxLimit / 10.0, 1);
    Serial.print("V)");
  }
  Serial.println();
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
    u32 value;
    DxlResult result = dxlReadValue(id, ADDR_PRESENT_POSITION, 4, &value);

    Serial.print(id);
    if (result == DXL_OK)
    {
      s32 position = (s32)value;
      Serial.print("\t");
      Serial.print(position);
      Serial.print("\t\t");
      Serial.print(positionToAngle(position), 1);
      Serial.println("°");
      connectedCount++;
    }
    else
    {
      Serial.print("\t[Error] ");
      Serial.println(dxlResultText(result));
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
  Serial.println("Enter angle (-180 to 180 degrees): ");

  String input = readInput();

  // 空欄はエラー（範囲チェックと送信は apiMove）
  if (input.length() == 0)
  {
    Serial.println("Error: Angle out of range (-180 to 180)");
    return;
  }

  apiMove(input.toFloat());
}

// [e]/[d]コマンド共通: 全サーボのトルク切替（各サーボの応答で成否確認）。全台成功ならtrue
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
    DxlResult result = dxlWriteValue(id, ADDR_TORQUE_ENABLE, 1, enable);

    Serial.print("ID ");
    Serial.print(id);
    if (result == DXL_OK)
    {
      Serial.println(enable ? ": Torque enabled" : ": Torque disabled");
      successCount++;
    }
    else
    {
      Serial.print(": Failed (");
      Serial.print(dxlResultText(result));
      Serial.println(")");
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
  Serial.println("Enter register address (0-65535): ");

  String addressInput = readInput();
  long address = addressInput.toInt();

  // アドレス範囲チェック
  if (addressInput.length() == 0 || !isDigit(addressInput[0]) || address < 0 || address > 65535)
  {
    Serial.println("Error: Address out of range (0-65535)");
    return;
  }

  Serial.println("Enter data length to read (1, 2 or 4 bytes): ");

  int dataLength = readInput().toInt();

  // データ長チェック
  if (dataLength != 1 && dataLength != 2 && dataLength != 4)
  {
    Serial.println("Error: Data length must be 1, 2 or 4");
    return;
  }

  Serial.print("Reading ");
  Serial.print(dataLength);
  Serial.print(" byte(s) from address ");
  Serial.print(address);
  Serial.print(" of servo ID ");
  Serial.println(targetServoID);

  u32 value;
  DxlResult result = dxlReadValue(targetServoID, address, dataLength, &value);
  if (result != DXL_OK)
  {
    Serial.print("Error: ");
    Serial.println(dxlResultText(result));
    return;
  }

  Serial.print("Register ");
  Serial.print(address);
  Serial.print(": Dec=");
  Serial.print(value);
  if (dataLength == 4)
  {
    Serial.print(" (signed ");
    Serial.print((s32)value);
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
  Serial.println("Enter register address (0-65535): ");

  String addressInput = readInput();
  long address = addressInput.toInt();

  // アドレス範囲チェック
  if (addressInput.length() == 0 || !isDigit(addressInput[0]) || address < 0 || address > 65535)
  {
    Serial.println("Error: Address out of range (0-65535)");
    return;
  }

  Serial.println("Enter data length to write (1, 2 or 4 bytes): ");

  int dataLength = readInput().toInt();

  // データ長チェック
  if (dataLength != 1 && dataLength != 2 && dataLength != 4)
  {
    Serial.println("Error: Data length must be 1, 2 or 4");
    return;
  }

  Serial.print("Enter decimal value to write (");
  if (dataLength == 1)
    Serial.print("0-255");
  else if (dataLength == 2)
    Serial.print("0-65535");
  else
    Serial.print("-2147483648 to 2147483647");
  Serial.println("): ");

  String valueInput = readInput();
  long value = valueInput.toInt();

  // 値範囲チェック
  if (valueInput.length() == 0 ||
      (dataLength == 1 && (value < 0 || value > 255)) ||
      (dataLength == 2 && (value < 0 || value > 65535)))
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

  DxlResult result = dxlWriteValue(targetServoID, address, dataLength, (u32)value);
  if (result != DXL_OK)
  {
    Serial.print("Write failed: ");
    Serial.println(dxlResultText(result));
    if (result == DXL_STATUS_ERROR && address < ADDR_EEPROM_END)
      Serial.println("Hint: EEPROM area (address 0-63) can be written only when torque is OFF ([d])");
    return;
  }
  Serial.println("Write command sent successfully");

  // 確認のため値を読み返す
  u32 readValue;
  result = dxlReadValue(targetServoID, address, dataLength, &readValue);
  if (result != DXL_OK)
  {
    Serial.print("Verification failed: ");
    Serial.println(dxlResultText(result));
    return;
  }

  u32 mask = (dataLength == 4) ? 0xFFFFFFFF : ((1UL << (8 * dataLength)) - 1);
  Serial.print("Verification: Read value = ");
  if (dataLength == 4)
    Serial.print((s32)readValue);
  else
    Serial.print(readValue);
  Serial.println(readValue == ((u32)value & mask) ? " [OK]" : " [MISMATCH]");
}

// [b]コマンド: 通信速度設定
void executeSetBaudRateCommand()
{
  Serial.print("Current ESP32 baud rate: ");
  Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  Serial.println("Dynamixel standard baud rates:");
  for (u8 i = 0; i < DXL_BAUD_COUNT; i++)
  {
    Serial.print("  [");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(DXL_BAUD_RATES[i]);
    Serial.println(" bps");
  }
  Serial.print("Enter baud rate number (0-");
  Serial.print(DXL_BAUD_COUNT - 1);
  Serial.println("): ");

  String baudInput = readInput();

  // 範囲チェック（toInt()は非数値で0を返すため数字であることも確認）
  int baudIndex = baudInput.toInt();
  if (baudInput.length() != 1 || !isDigit(baudInput[0]) || baudIndex >= (int)DXL_BAUD_COUNT)
  {
    Serial.println("Error: Baud rate number out of range");
    return;
  }

  Serial.println("Apply to: [1] ESP32 only");
  Serial.print("          [2] Servo ID ");
  Serial.print(targetServoID);
  Serial.println(" (EEPROM) + ESP32");

  String modeInput = readInput();

  // [1] ESP32だけ / [2] サーボ + ESP32（処理は apiSetBaud）
  if (modeInput == "1" || modeInput == "2")
    apiSetBaud(baudIndex, modeInput == "2");
  else
    Serial.println("Error: Invalid selection");
}

// [i]コマンド: サーボID変更
void executeChangeServoIdCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);

  // 変更元サーボの存在確認
  if (dxlPing(targetServoID, NULL, NULL, RESPONSE_TIMEOUT) != DXL_OK)
  {
    Serial.println("Error: No response from target servo (use [z] to select ID)");
    return;
  }

  Serial.println("Enter new servo ID (0-252): ");

  String input = readInput();

  // 数字でなければエラー（範囲チェックと変更は apiSetId）
  if (input.length() == 0 || !isDigit(input[0]))
  {
    Serial.println("Error: ID out of range (0-252)");
    return;
  }

  apiSetId(targetServoID, input.toInt());
}

// [m]コマンド: サーボのバージョン情報（型番/ファームウェア）読み取り
void executeReadVersionCommand()
{
  Serial.print("Reading version info from servo ID ");
  Serial.println(targetServoID);

  // アドレス0から7バイト読み取り（型番2, 型番情報4, FW版1）
  u8 data[7];
  DxlResult result = dxlRead(targetServoID, ADDR_MODEL_NUMBER, 7, data);
  if (result != DXL_OK)
  {
    Serial.print("Error: ");
    Serial.println(dxlResultText(result));
    return;
  }

  u16 modelNumber = getValue(&data[0], 2);
  u32 modelInformation = getValue(&data[ADDR_MODEL_INFORMATION], 4);
  u8 firmwareVersion = data[ADDR_FIRMWARE_VERSION];

  Serial.println("Protocol        : 2.0");
  Serial.print("Model number    : ");
  Serial.print(modelNumber);
  Serial.print(" -> ");
  Serial.print(modelName(modelNumber));
  Serial.println(isSupportedModel(modelNumber) ? "" : " [Not supported by this tool]");
  Serial.print("Model info      : 0x");
  Serial.println(modelInformation, HEX);
  Serial.print("Firmware version: ");
  Serial.println(firmwareVersion);

  // ハードウェアエラー状態
  u32 hwError;
  if (dxlReadValue(targetServoID, ADDR_HARDWARE_ERROR_STATUS, 1, &hwError) == DXL_OK)
  {
    Serial.print("Hardware error  : ");
    if (hwError == 0)
    {
      Serial.println("None");
    }
    else
    {
      Serial.print("0x");
      Serial.print(hwError, HEX);
      if (hwError & 0x01)
        Serial.print(" InputVoltage");
      if (hwError & 0x04)
        Serial.print(" Overheating");
      if (hwError & 0x08)
        Serial.print(" MotorEncoder");
      if (hwError & 0x10)
        Serial.print(" ElectricalShock");
      if (hwError & 0x20)
        Serial.print(" Overload");
      Serial.println(" (power cycle to clear)");
    }
  }
}

// [t]コマンド: 送信テスト（ブロードキャストで全IDにトルクONと位置指令を送る。受信不要でTX側の確認用）
void executeTxTestCommand()
{
  Serial.println("TX test: broadcasting torque ON and position to ALL IDs: +30, -30, +30, -30, center.");
  Serial.println("No reception is needed. If the servo moves, TX (ESP32 -> servo) is OK.");
  Serial.println("(Position control mode is required.)");

  const u8 torqueOn = 1;
  dxlWrite(BROADCAST_ID, ADDR_TORQUE_ENABLE, &torqueOn, 1);
  delay(10);

  const float angles[] = {30.0, -30.0, 30.0, -30.0, 0.0};
  for (u8 step = 0; step < sizeof(angles) / sizeof(angles[0]); step++)
  {
    // Profile Acceleration(108) + Profile Velocity(112) + Goal Position(116)
    u8 data[12];
    putU32(&data[0], MOVE_PROFILE_ACCELERATION);
    putU32(&data[4], MOVE_PROFILE_VELOCITY);
    putU32(&data[8], (u32)angleToPosition(angles[step]));
    dxlWrite(BROADCAST_ID, ADDR_PROFILE_ACCELERATION, data, sizeof(data));

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
  Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps, Protocol: 2.0");
  Serial.println("Menu: [f] Find servos   [n] Set sync servo IDs   [z] Set target ID ");
  Serial.println("      [v] Read voltage  [r] Read position        [p] Read register");
  Serial.println("      [c] All to center [a] Read all positions   [s] Write register");
  Serial.println("      [e] All torque ON [d] All torque OFF       [x] Send angle to all ");
  Serial.println("      [b] Set baud rate [i] Change servo ID      [m] Read version");
  Serial.println("      [t] TX test (all IDs) [h] Change servo type (restart)");
  Serial.println("Input Command:");
}

// 初期化
void setup()
{
  Serial.begin(115200);
  // サーボバス開始（8N1, エコー自動判定, 送信モードで待機）
  busBegin(&Serial1, RX1, TX1, PIN_EN, DXL_BAUD_RATES[servoBaudIndex], SERIAL_8N1);

  delay(1000);
  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());

  Serial.println("=====================================");
  Serial.println("Dynamixel Servo Setter");
  Serial.println("Protocol 2.0 (X series / MX(2.0) series)");
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

// ===== サーボ種類の自動判別（起動時のメニュー [0]） =====

// 読み取りコマンドだけでProtocol 2.0 のサーボを探す（サーボは動かない）。
// defaultBaud: true = 工場出荷時の通信速度だけ, false = それ以外の速度。見つかった通信速度（なければ0）
unsigned long detect(bool defaultBaud)
{
  const u8 original = servoBaudIndex;
  unsigned long found = 0;
  busBegin(servoSerial, RX1, TX1, PIN_EN, DXL_BAUD_RATES[DEFAULT_BAUD_INDEX], SERIAL_8N1);
  for (int b = DXL_BAUD_COUNT - 1; b >= 0 && found == 0; b--)
  {
    if ((b == DEFAULT_BAUD_INDEX) != defaultBaud)
      continue;
    setHostBaudRate(b);
    for (int id = 0; id <= DEFAULT_SCAN_MAX_ID; id++)
    {
      if (dxlPing(id, NULL, NULL, detectTimeout(DXL_BAUD_RATES[b], 14)) == DXL_OK)
      {
        found = DXL_BAUD_RATES[b];
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
  for (u8 i = 0; i < DXL_BAUD_COUNT; i++)
  {
    if (DXL_BAUD_RATES[i] == baud)
      servoBaudIndex = i;
  }
}

// ===== ブラウザ用API =====

// 今の状態と、この種類でできること
void apiGetInfo(ApiInfo *info)
{
  info->idMin = 0;
  info->idMax = DXL_ID_MAX;
  info->angleMin = -180.0;
  info->angleMax = 180.0;
  info->bauds = DXL_BAUD_RATES;
  info->baudCount = DXL_BAUD_COUNT;
  info->baudIndex = servoBaudIndex;
  info->baud = DXL_BAUD_RATES[servoBaudIndex];
  info->target = targetServoID;
  info->syncIds = syncServoIDs;
  info->syncCount = syncServoCount;
  info->oneServoForId = false;
}

// [f]と同じ検索（現在の通信速度のみ、全ボーレート検索はしない）。見つかった数を返す
int apiScan(int maxId, ApiServo *found, int maxCount)
{
  static u8 foundServos[DXL_ID_MAX + 1];
  static u16 foundModels[DXL_ID_MAX + 1];

  if (maxId < 0 || maxId > DXL_ID_MAX)
  {
    Serial.println("Error: ID out of range (0-252)");
    return -1;
  }

  u8 foundCount = scanAndApply(maxId, foundServos, foundModels);
  if (foundCount == 0)
  {
    Serial.print("No servos detected at ");
    Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
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

// [z]と同じ: ターゲットID変更（範囲外はfalse）
bool apiSetTarget(int id)
{
  if (id < 0 || id > DXL_ID_MAX)
    return false;

  targetServoID = id;
  Serial.print("Target ID changed to: ");
  Serial.println(targetServoID);
  return true;
}

// [n]と同じ: 一斉制御リスト設定（昇順・重複なしに整える。範囲外のIDや空リストはfalse）
bool apiSetSync(const uint8_t *ids, uint8_t count)
{
  bool selected[DXL_ID_MAX + 1] = {false};
  for (u8 i = 0; i < count; i++)
  {
    if (ids[i] > DXL_ID_MAX)
      return false;
    selected[ids[i]] = true;
  }

  u8 newCount = 0;
  for (int id = 0; id <= DXL_ID_MAX; id++)
  {
    if (selected[id])
      syncServoIDs[newCount++] = id;
  }
  if (newCount == 0)
    return false;
  syncServoCount = newCount;

  Serial.print("Sync servo IDs changed to: ");
  printSyncServoIDs();
  Serial.print(" (");
  Serial.print(syncServoCount);
  Serial.println(" servos)");
  return true;
}

// [x]と同じ: 一斉制御リストの全サーボを角度へ（トルクONも行う）
bool apiMove(float deg)
{
  // 角度範囲チェック
  if (!(deg >= -180.0 && deg <= 180.0))
  {
    Serial.println("Error: Angle out of range (-180 to 180)");
    return false;
  }

  s32 position = angleToPosition(deg);
  Serial.print("Converting ");
  Serial.print(deg);
  Serial.print(" degrees to position ");
  Serial.println(position);

  if (!moveSyncServos(position, MOVE_PROFILE_VELOCITY))
    return false;
  Serial.println("Angle command sent successfully");
  return true;
}

// [c]と同じ: 一斉制御リストの全サーボを中央へ
bool apiCenter()
{
  Serial.print("Resetting ");
  Serial.print(syncServoCount);
  Serial.println(" servos to position 0 (center)...");

  if (!moveSyncServos(POSITION_CENTER, CENTER_PROFILE_VELOCITY))
    return false;
  Serial.println("All servos reset to center position");
  return true;
}

// [e]/[d]と同じ: 全サーボのトルク切替（全台成功ならtrue）
bool apiTorque(bool on)
{
  return setTorqueAll(on ? 1 : 0);
}

// [t]と同じ: 送信テスト（応答を見ないので常にtrue）
bool apiTxTest()
{
  executeTxTestCommand();
  return true;
}

// [r]と同じ: 現在位置を読み取り、角度(度)で返す
bool apiReadPosition(int id, float *deg)
{
  if (id < 0 || id > DXL_ID_MAX)
    return false;

  Serial.print("Reading position from servo ID ");
  Serial.println(id);

  u32 value;
  DxlResult result = dxlReadValue(id, ADDR_PRESENT_POSITION, 4, &value);
  if (result != DXL_OK)
  {
    Serial.print("Error: ");
    Serial.println(dxlResultText(result));
    return false;
  }

  s32 position = (s32)value;
  *deg = positionToAngle(position);
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" (");
  Serial.print(*deg, 1);
  Serial.println("°)");
  return true;
}

// [i]と同じ: サーボID変更（トルクOFF → 書き込み → 新IDで応答確認）。成功するとターゲットIDも新IDになる
bool apiSetId(int oldId, int newId)
{
  // ID範囲チェック
  if (oldId < 0 || oldId > DXL_ID_MAX || newId < 0 || newId > DXL_ID_MAX)
  {
    Serial.println("Error: ID out of range (0-252)");
    return false;
  }

  // 変更元サーボの存在確認
  if (dxlPing(oldId, NULL, NULL, RESPONSE_TIMEOUT) != DXL_OK)
  {
    Serial.println("Error: No response from target servo (use [z] to select ID)");
    return false;
  }

  if (newId == oldId)
  {
    Serial.println("New ID is the same as current ID. Nothing to do.");
    return true;
  }

  // ID重複チェック
  if (dxlPing(newId, NULL, NULL, SCAN_TIMEOUT) != DXL_NO_RESPONSE)
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

  // EEPROM書き込みのためトルクOFF
  DxlResult result = dxlWriteValue(oldId, ADDR_TORQUE_ENABLE, 1, 0);
  if (result != DXL_OK)
  {
    Serial.print("Error: Failed to disable torque (");
    Serial.print(dxlResultText(result));
    Serial.println(")");
    return false;
  }

  // ID書き込み（応答IDが新旧どちらでも新IDのPINGで最終確認する）
  result = dxlWriteValue(oldId, ADDR_ID, 1, newId);
  if (result == DXL_STATUS_ERROR)
  {
    Serial.print("Error: ");
    Serial.println(dxlResultText(result));
    return false;
  }
  delay(50);

  // 新IDで応答確認
  if (dxlPing(newId, NULL, NULL, RESPONSE_TIMEOUT) != DXL_OK)
  {
    Serial.println("Verification failed: No response at new ID");
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
  Serial.print("Servo ID changed successfully [OK]. Target ID is now ");
  Serial.println(targetServoID);
  return true;
}

// [b]と同じ: 通信速度変更（servoToo: ターゲットサーボ(EEPROM) + ESP32, false: ESP32だけ）
bool apiSetBaud(int index, bool servoToo)
{
  // 範囲チェック
  if (index < 0 || index >= (int)DXL_BAUD_COUNT)
  {
    Serial.println("Error: Baud rate number out of range");
    return false;
  }

  if (!servoToo)
  {
    setHostBaudRate(index);
    Serial.print("ESP32 baud rate changed to: ");
    Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
    Serial.println(" bps");
    return true;
  }

  // EEPROM書き込みのためトルクOFF
  DxlResult result = dxlWriteValue(targetServoID, ADDR_TORQUE_ENABLE, 1, 0);
  if (result != DXL_OK)
  {
    Serial.print("Error: Failed to disable torque (");
    Serial.print(dxlResultText(result));
    Serial.println(")");
    return false;
  }

  Serial.print("Writing baud rate ");
  Serial.print(DXL_BAUD_RATES[index]);
  Serial.print(" bps to servo ID ");
  Serial.println(targetServoID);

  // ボーレート書き込み（応答は旧速度で返る想定。応答なしでも続行）
  result = dxlWriteValue(targetServoID, ADDR_BAUD_RATE, 1, index);
  if (result == DXL_STATUS_ERROR)
  {
    Serial.print("Error: ");
    Serial.println(dxlResultText(result));
    return false;
  }
  delay(50);

  // ESP32側も新速度へ追従
  setHostBaudRate(index);

  // 新しい速度で読み返して確認
  Serial.println("Verifying at new baud rate...");
  bool verified = false;
  u32 readValue;
  result = dxlReadValue(targetServoID, ADDR_BAUD_RATE, 1, &readValue);
  if (result == DXL_OK)
  {
    verified = (readValue == (u32)index);
    Serial.print("Verification: Baud rate register = ");
    Serial.print(readValue);
    Serial.println(verified ? " [OK]" : " [MISMATCH]");
  }
  else
  {
    Serial.println("Verification failed: No response at new baud rate");
    Serial.println("Hint: Use [b] -> [1] to try other ESP32 baud rates");
  }

  Serial.print("ESP32 baud rate: ");
  Serial.print(DXL_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  return verified;
}

} // namespace dynamixel
