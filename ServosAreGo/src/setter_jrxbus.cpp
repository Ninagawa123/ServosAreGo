#include <Arduino.h>
#include "servo_bus.h"
#include <xtensa/hal.h>
#include "servo_select.h"

namespace jrxbus
{

// 基本型定義
typedef unsigned char u8;
typedef unsigned short u16;

// ピン定義
#define PIN_EN 33 // 半二重通信の送受信切替（HIGH: 送信, LOW: 受信）

// XBUS通信定義
#define XBUS_BAUD_RATE 250000 // 通信速度（固定, 8N1）
#define XBUS_INTERVAL 14      // チャンネルデータの送信周期(ms)。途切れると約1.5秒でサーボが脱力

// XBUSコマンド定義
#define XBUS_CMD_SET 0x20    // 設定（RAM, ParamWriteで保存）
#define XBUS_CMD_GET 0x21    // 取得
#define XBUS_CMD_STATUS 0x22 // ステータス（応答）
#define XBUS_CMD_MODE_A 0xA4 // チャンネルデータ（位置指令）

// XBUSオーダー定義（JR PROPO XBusServoライブラリより）
#define ORDER_MODE 0x01         // 1バイト: 1=通常動作, 2=ID設定
#define ORDER_ID 0x03           // 1バイト
#define ORDER_VERSION 0x04      // 2バイト（取得のみ）
#define ORDER_PRODUCT 0x05      // 2バイト（取得のみ）
#define ORDER_UNSUPPORTED 0x06  // ステータス応答のみ（未対応オーダー）
#define ORDER_PARAM_RESET 0x07  // 2バイト（設定のみ, 値=パラメータ番号を初期値に戻す）
#define ORDER_PARAM_WRITE 0x08  // 2バイト（設定のみ, 値=パラメータ番号で保存）
#define ORDER_REVERSE 0x10      // 2バイト
#define ORDER_CURRENT_POS 0x20  // 2バイト（取得のみ）
#define ORDER_ANGLE_180 0x1D    // 1バイト: 0=通常(±60度), 1=180度モード(±90度)
#define ORDER_CURRENT_POW 0x21  // 1バイト（取得のみ, 0-100%）

// XBUSモード定義
#define XBUS_MODE_OPERATE 0x01
#define XBUS_MODE_ID_SET 0x02

// パラメータ番号（ParamWrite / ParamResetの値）
#define PARAM_IDX_ALL_WITH_ID 0x0001 // 全パラメータ+ID（リセット専用, CH-ID 0x00で送信, 自動保存）
#define PARAM_IDX_REVERSED 0x0004

// ID定義
#define XBUS_BROADCAST_ID 0x00 // 全サーボ（Setのみ, 応答なし）
#define XBUS_ID_MIN 1
#define XBUS_ID_MAX 50
#define XBUS_SUB_ID_MASK 0x3F // チャンネルIDの下位6ビットがID（上位2ビットはサブID, 本ツールは0固定）

// 位置定義（16ビット: 0x0000=800us, 0x7FFF=1500us, 0xFFFF=2200us）
#define XBUS_POSITION_NEUTRAL 0x7FFF
#define XBUS_POSITION_MIN 0x1249  // 900us（通常-60度 / 180度モード-90度）
#define XBUS_POSITION_MAX 0xEDB6  // 2100us（通常+60度 / 180度モード+90度）
#define XBUS_POSITION_RELEASE 0x0000 // 位置0で即座に脱力
#define XBUS_PULSE_HALF_US 600.0 // 中央から900us/2100usまでの幅
#define DEG_PER_US_NORMAL 0.10 // 通常モード: 900-2100usで±60度
#define DEG_PER_US_180 0.15    // 180度モード: 900-2100usで±90度

// 一斉制御定義
#define MAX_SYNC_SERVOS 50
#define MAX_PACKET_SIZE (4 + MAX_SYNC_SERVOS * 4 + 1)

// 通信タイムアウト定義
#define RESPONSE_TIMEOUT 100 // 通常の応答待ち(ms)
#define SCAN_TIMEOUT 30      // サーボ検索時の応答待ち(ms)

// サーボ検索のデフォルト最大ID（[f]で空欄リターン時）
#define DEFAULT_SCAN_MAX_ID 20

// 既知のサーボ型番（XBusサーボ プロトコル仕様概略 Ver.1.0.6 別表2より）
struct XBusModel
{
  u16 number;
  const char *name;
};
const XBusModel XBUS_MODELS[] = {
    {0x0200, "NX8921"},
    {0x0201, "NX3421"},
    {0x0202, "NX588"},
    {0x0203, "NX8925"},
    {0x0204, "NX3425"},
    {0x0205, "NX6421"},
    {0x0206, "NXR89"},
    {0x0207, "NXR34"},
    {0x0208, "NXB8921"},
    {0x0209, "NXB8925"},
    {0x020A, "NXB89G"},
    {0x020B, "NX8931"},
    {0x020C, "NX8935"},
    {0x020E, "NX35G"},
    {0x020F, "NX396"},
    {0x0210, "NX319"},
    {0x0211, "NX189"},
};
#define XBUS_MODEL_COUNT (sizeof(XBUS_MODELS) / sizeof(XBUS_MODELS[0]))

// オーダー一覧（[p]/[s]用。範囲はXBusサーボ プロトコル仕様概略 Ver.1.0.6より）
struct XBusOrderInfo
{
  u8 order;
  const char *name;
  u8 size;         // データサイズ（1 or 2バイト）
  u16 paramIndex;  // ParamWrite用のパラメータ番号（0は保存不可/不要）
  bool writable;   // 設定可能か
  long minValue;
  long maxValue;
};
const XBusOrderInfo XBUS_ORDERS[] = {
    {ORDER_VERSION, "Version", 2, 0, false, 0, 0},
    {ORDER_PRODUCT, "Product", 2, 0, false, 0, 0},
    {ORDER_REVERSE, "Reverse", 2, 0x0004, true, 0, 1},
    {0x11, "Neutral", 2, 0x0005, true, -600, 600},
    {0x12, "H_Travel", 2, 0x0006, true, 0, 192},
    {0x13, "L_Travel", 2, 0x0007, true, 0, 192},
    {0x14, "H_Limit", 2, 0x0008, true, 0, 0xFFFF},
    {0x15, "L_Limit", 2, 0x0009, true, 0, 0xFFFF},
    {0x16, "P_Gain", 1, 0x000A, true, -50, 50},
    {0x17, "I_Gain", 1, 0x000B, true, -50, 50},
    {0x18, "D_Gain", 1, 0x000C, true, -50, 50},
    {0x19, "DeadBand", 1, 0x000D, true, -10, 10},
    {0x1A, "Boost", 2, 0x000E, true, -999, 999},
    {0x1B, "AlarmLevel", 1, 0x000F, true, 0, 100},
    {0x1C, "AlarmDelay", 2, 0x0010, true, 0, 5000},
    {ORDER_ANGLE_180, "Angle_180", 1, 0x0011, true, 0, 1},
    {0x1E, "SlowStart", 1, 0x0012, true, 0, 1},
    {0x1F, "StopMode", 1, 0x0013, true, 0, 1},
    {ORDER_CURRENT_POS, "CurrentPos", 2, 0, false, 0, 0},
    {ORDER_CURRENT_POW, "CurrentPow", 1, 0, false, 0, 0},
    {0x22, "SpeedLimit", 1, 0x0014, true, 0, 30},
    {0x23, "MaxInteger", 2, 0x0015, true, -999, 999},
};
#define XBUS_ORDER_COUNT (sizeof(XBUS_ORDERS) / sizeof(XBUS_ORDERS[0]))

// 通信結果
enum XBusResult
{
  XBUS_OK,
  XBUS_NO_RESPONSE,
  XBUS_UNSUPPORTED
};

// グローバル変数
HardwareSerial *servoSerial = &Serial1;                 // 使用するシリアルポート
u8 targetServoID = 1;                                   // ターゲットサーボID
u8 syncServoIDs[MAX_SYNC_SERVOS] = {1, 2, 3, 4, 5, 6}; // 一斉制御するサーボIDリスト（とびとびIDも可）
u8 syncServoCount = 6;                                  // リスト内のサーボ数
u16 syncTargets[MAX_SYNC_SERVOS];                       // 各サーボの目標位置（チャンネルデータで送信）
volatile bool outputEnabled = false;                    // チャンネルデータ送信中か（ON = 保持）
SemaphoreHandle_t busMutex;                             // バス排他（送信タスクとコマンドで共有）

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

// 数値入力の解析（10進数, 負数, 0x付き16進数に対応）。成功時true
bool parseNumber(String input, long *value)
{
  input.trim();
  if (input.length() == 0)
    return false;

  if (input.startsWith("0x") || input.startsWith("0X"))
  {
    if (input.length() == 2)
      return false;
    for (unsigned int i = 2; i < input.length(); i++)
    {
      if (!isHexadecimalDigit(input[i]))
        return false;
    }
    *value = strtol(input.c_str() + 2, NULL, 16);
    return true;
  }

  for (unsigned int i = 0; i < input.length(); i++)
  {
    if (i == 0 && input[i] == '-' && input.length() > 1)
      continue;
    if (!isDigit(input[i]))
      return false;
  }
  *value = input.toInt();
  return true;
}

// 型番名（未知の型番は"Unknown"）
const char *modelName(u16 modelNumber)
{
  for (u8 i = 0; i < XBUS_MODEL_COUNT; i++)
  {
    if (XBUS_MODELS[i].number == modelNumber)
      return XBUS_MODELS[i].name;
  }
  return "Unknown";
}

// オーダー情報の検索（見つからなければNULL）
const XBusOrderInfo *findOrder(u8 order)
{
  for (u8 i = 0; i < XBUS_ORDER_COUNT; i++)
  {
    if (XBUS_ORDERS[i].order == order)
      return &XBUS_ORDERS[i];
  }
  return NULL;
}

// 通信結果の表示用テキスト
const char *xbusResultText(XBusResult result)
{
  switch (result)
  {
  case XBUS_OK:
    return "OK";
  case XBUS_NO_RESPONSE:
    return "No response";
  default:
    return "Unsupported order";
  }
}

// 位置値から角度への変換（仕様の基準点 0x1249/0x7FFF/0xEDB6 = 900/1500/2100us を直線で結ぶ）
float positionToAngle(u16 position, bool angle180)
{
  float span = (position >= XBUS_POSITION_NEUTRAL) ? (XBUS_POSITION_MAX - XBUS_POSITION_NEUTRAL)
                                                   : (XBUS_POSITION_NEUTRAL - XBUS_POSITION_MIN);
  float pulseOffsetUs = ((float)position - XBUS_POSITION_NEUTRAL) / span * XBUS_PULSE_HALF_US;
  return pulseOffsetUs * (angle180 ? DEG_PER_US_180 : DEG_PER_US_NORMAL);
}

// 角度から位置値への変換（900-2100usの範囲に制限）
u16 angleToPosition(float angle, bool angle180)
{
  float pulseOffsetUs = angle / (angle180 ? DEG_PER_US_180 : DEG_PER_US_NORMAL);
  pulseOffsetUs = constrain(pulseOffsetUs, -XBUS_PULSE_HALF_US, XBUS_PULSE_HALF_US);
  float span = (pulseOffsetUs >= 0) ? (XBUS_POSITION_MAX - XBUS_POSITION_NEUTRAL)
                                    : (XBUS_POSITION_NEUTRAL - XBUS_POSITION_MIN);
  long position = XBUS_POSITION_NEUTRAL + lroundf(pulseOffsetUs / XBUS_PULSE_HALF_US * span);
  return constrain(position, XBUS_POSITION_MIN, XBUS_POSITION_MAX);
}

// ===== XBUS 通信レイヤー =====

// CRC-8計算（JR PROPO XBusServoライブラリのテーブルと同等: 反転多項式0x8C, 初期値0）
u8 crc8(const u8 *data, u16 length)
{
  u8 crc = 0;
  for (u16 i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (u8 bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x01) ? (crc >> 1) ^ 0x8C : (crc >> 1);
    }
  }
  return crc;
}

// バス排他の取得・解放
void lockBus()
{
  xSemaphoreTake(busMutex, portMAX_DELAY);
}

void unlockBus()
{
  xSemaphoreGive(busMutex);
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
}

// 半二重通信でパケット送信し、送信完了後すぐ（数us以内に）受信モードに切り替える
void sendPacketToRx(const u8 *packet, u16 length)
{
  flushReceiveBuffer();
  busDirTx();
  servoSerial->write(packet, length);
  busEndSendToRx();
}

// コマンドパケット構築（cmd, len, key, chID, order, data(1-2), CRC）。戻り値はパケット長
u8 buildCommandPacket(u8 *packet, u8 command, u8 channelID, u8 order, long value, u8 valueSize)
{
  packet[0] = command;
  packet[1] = valueSize + 3; // key + chID + order + data
  packet[2] = 0x00;          // key
  packet[3] = channelID;
  packet[4] = order;
  if (valueSize == 1)
  {
    packet[5] = value & 0xFF;
  }
  else
  {
    packet[5] = (value >> 8) & 0xFF; // 2バイト値はビッグエンディアン
    packet[6] = value & 0xFF;
  }

  u8 length = packet[1] + 2;
  packet[length] = crc8(packet, length);
  return length + 1;
}

// 応答受信（Status 0x22のみ応答として扱う。送信エコー等は読み飛ばし、CRC・ID・オーダーを検証）
XBusResult receiveResponse(u8 channelID, u8 order, u8 valueSize, long *value, unsigned long timeout)
{
  u8 buffer[16];
  u8 count = 0;
  unsigned long startTime = millis();

  while ((millis() - startTime) < timeout)
  {
    if (!servoSerial->available())
      continue;

    buffer[count++] = servoSerial->read();

    // バッファ先頭からパケットを解析（不正なら1バイトずつずらして再同期）
    while (count >= 2)
    {
      u8 command = buffer[0];
      u8 length = buffer[1]; // 受信長は必ず2バイト目で判断（未対応応答は1バイト短い）
      bool validHeader = (command >= XBUS_CMD_SET && command <= XBUS_CMD_STATUS) && length >= 3 && length <= 8;

      if (validHeader && count < length + 3)
        break; // パケット受信途中

      u8 total = length + 3;
      bool accepted = false;
      if (validHeader && crc8(buffer, total) == 0)
      {
        if (command == XBUS_CMD_STATUS && buffer[3] == channelID)
        {
          if (buffer[4] == ORDER_UNSUPPORTED)
            return XBUS_UNSUPPORTED;
          if (buffer[4] == order)
          {
            if (value)
            {
              if (valueSize == 1)
                *value = (long)(signed char)buffer[5];
              else
                *value = ((long)buffer[5] << 8) | buffer[6];
            }
            return XBUS_OK;
          }
        }
        accepted = true; // 送信エコーや他サーボ宛ての応答は読み飛ばす
      }

      // 処理済みパケット（または不正バイト）を取り除く
      u8 shift = accepted ? total : 1;
      memmove(buffer, buffer + shift, count - shift);
      count -= shift;
    }

    if (count >= sizeof(buffer))
      count = 0;
  }

  return XBUS_NO_RESPONSE;
}

// コマンド送信→応答受信（valueは送信値, 応答値で上書き）
XBusResult xbusCommand(u8 command, u8 channelID, u8 order, long *value, u8 valueSize,
                       unsigned long timeout = RESPONSE_TIMEOUT)
{
  u8 packet[8];
  u8 length = buildCommandPacket(packet, command, channelID, order, value ? *value : 0, valueSize);

  lockBus();

  // ブロードキャスト（CH-ID 0x00）は応答なし（送信モードを維持）
  if (channelID == XBUS_BROADCAST_ID)
  {
    sendPacket(packet, length);
    unlockBus();
    return XBUS_OK;
  }

  // 送信 → すぐ受信モードに切り替え → 送信エコーを読み捨てて応答を受信
  sendPacketToRx(packet, length);
  busSkipEcho(packet, length, 10);
  XBusResult result = receiveResponse(channelID, order, valueSize, value, timeout);

  // 送信モードに復帰
  busDirTx();
  unlockBus();

  return result;
}

// オーダーのデータサイズ（未知のオーダーは1バイト）
u8 orderSize(u8 order)
{
  if (order == ORDER_ID || order == ORDER_MODE)
    return 1;
  if (order == ORDER_PARAM_WRITE || order == ORDER_PARAM_RESET)
    return 2;
  const XBusOrderInfo *info = findOrder(order);
  return info ? info->size : 1;
}

// GET（取得）
XBusResult xbusGet(u8 id, u8 order, long *value, unsigned long timeout = RESPONSE_TIMEOUT)
{
  *value = 0;
  return xbusCommand(XBUS_CMD_GET, id, order, value, orderSize(order), timeout);
}

// SET（設定, RAM）
XBusResult xbusSet(u8 id, u8 order, long value)
{
  return xbusCommand(XBUS_CMD_SET, id, order, &value, orderSize(order));
}

// 設定値を保存（ParamWrite: 値 = パラメータ番号）
XBusResult xbusParamWrite(u8 id, u16 paramIndex)
{
  return xbusSet(id, ORDER_PARAM_WRITE, paramIndex);
}

// PING相当（型番を取得して存在確認）
XBusResult xbusPing(u8 id, u16 *modelNumber, unsigned long timeout)
{
  long value;
  XBusResult result = xbusGet(id, ORDER_PRODUCT, &value, timeout);
  if (result == XBUS_OK && modelNumber)
    *modelNumber = value & 0xFFFF;
  return result;
}

// チャンネルデータパケット送信（0xA4, len, key, type, {ID, 0, 位置H, 位置L}×N, CRC）
void sendChannelData()
{
  static u8 packet[MAX_PACKET_SIZE];
  u16 index = 0;

  packet[index++] = XBUS_CMD_MODE_A;
  packet[index++] = syncServoCount * 4 + 2; // key + type + データ
  packet[index++] = 0x00;                   // key
  packet[index++] = 0x00;                   // type
  for (u8 i = 0; i < syncServoCount; i++)
  {
    packet[index++] = syncServoIDs[i];
    packet[index++] = 0x00;
    packet[index++] = syncTargets[i] >> 8;
    packet[index++] = syncTargets[i] & 0xFF;
  }
  packet[index] = crc8(packet, index);
  index++;

  sendPacket(packet, index);
}

// チャンネルデータ送信タスク（ON中は14ms周期で送り続ける）
void channelOutputTask(void *parameter)
{
  TickType_t lastWake = xTaskGetTickCount();
  for (;;)
  {
    if (outputEnabled)
    {
      lockBus();
      sendChannelData();
      unlockBus();
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(XBUS_INTERVAL));
  }
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

// 一斉制御リストを置き換え（出力を止めて目標を中央に初期化）
void setSyncServoList(const u8 *ids, u8 count)
{
  lockBus();
  outputEnabled = false;
  memcpy(syncServoIDs, ids, count);
  syncServoCount = count;
  for (u8 i = 0; i < count; i++)
    syncTargets[i] = XBUS_POSITION_NEUTRAL;
  unlockBus();
}

// IDリスト文字列を解析（例: "1,3,7" / "1-4,7"）。昇順・重複なしでoutに格納し、個数を返す（-1はエラー）
int parseIdList(String input, u8 *out)
{
  bool selected[XBUS_ID_MAX + 1] = {false};
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
    if (first < XBUS_ID_MIN || last > XBUS_ID_MAX || first > last)
      return -1;

    for (int id = first; id <= last; id++)
      selected[id] = true;
  }

  int count = 0;
  for (int id = XBUS_ID_MIN; id <= XBUS_ID_MAX; id++)
  {
    if (selected[id])
      out[count++] = id;
  }
  return count > 0 ? count : -1;
}

// 一斉制御リストの各サーボの目標位置を設定し、出力ON
void moveSyncServos(const u16 *positions)
{
  lockBus();
  memcpy(syncTargets, positions, syncServoCount * sizeof(u16));
  outputEnabled = true;
  unlockBus();
}

// 180度モード（Angle_180=1）か。取得できなければ通常モード扱い
bool isAngle180(u8 id)
{
  long value;
  return xbusGet(id, ORDER_ANGLE_180, &value) == XBUS_OK && value == 1;
}

// ===== コマンド =====

bool sweepIdRange(u8 firstId, u8 lastId, unsigned long duration, bool stopOnInput);

// 指定範囲のIDを検索。見つかった数を返す
u8 scanServos(int maxId, u8 *foundServos, u16 *foundModels)
{
  u8 foundCount = 0;

  // プログレス表示用
  int progressStep = maxId / 10;
  if (progressStep == 0)
    progressStep = 1;

  for (int id = XBUS_ID_MIN; id <= maxId; id++)
  {
    // プログレス表示
    if (id % progressStep == 0 || id == maxId)
    {
      Serial.print("Progress: ");
      Serial.print((id * 100) / maxId);
      Serial.println("%");
    }

    u16 modelNumber;
    if (xbusPing(id, &modelNumber, SCAN_TIMEOUT) == XBUS_OK)
    {
      long version = 0;
      xbusGet(id, ORDER_VERSION, &version, SCAN_TIMEOUT);

      foundServos[foundCount] = id;
      foundModels[foundCount] = modelNumber;
      foundCount++;

      Serial.print("Found servo ID: ");
      Serial.print(id);
      Serial.print(" (");
      Serial.print(modelName(modelNumber));
      Serial.print(", product 0x");
      Serial.print(modelNumber, HEX);
      Serial.print(", version 0x");
      Serial.print(version & 0xFFFF, HEX);
      Serial.println(")");
    }

    delay(2); // 次のスキャンまで少し待機
  }

  return foundCount;
}

// 見つかったIDにターゲットとリストを自動設定（出力はOFFに戻る）
void applyFoundServos(const u8 *ids, u8 count)
{
  setSyncServoList(ids, count);
  targetServoID = syncServoIDs[0];
}

// [f]コマンド: サーボ認識
void executeServoDetectionCommand()
{
  static u8 foundServos[MAX_SYNC_SERVOS];
  static u16 foundModels[MAX_SYNC_SERVOS];

  Serial.print("Enter maximum ID to scan (1-50, empty = ");
  Serial.print(DEFAULT_SCAN_MAX_ID);
  Serial.println("): ");

  String input = readInput();

  // 空欄リターンはデフォルト最大IDで検索
  long maxId = DEFAULT_SCAN_MAX_ID;
  if (input.length() > 0 && !parseNumber(input, &maxId))
    maxId = -1;

  // ID範囲チェック
  if (maxId < XBUS_ID_MIN || maxId > XBUS_ID_MAX)
  {
    Serial.println("Error: ID out of range (1-50)");
    return;
  }

  Serial.print("Scanning servos from ID1 to ID");
  Serial.print(maxId);
  Serial.print(" at ");
  Serial.print(XBUS_BAUD_RATE);
  Serial.println(" bps...");
  Serial.println();

  u8 foundCount = scanServos(maxId, foundServos, foundModels);

  // 結果表示
  Serial.println();
  Serial.println("=== Servo Detection Results ===");
  Serial.print("Scanned range: ID1 to ID");
  Serial.println(maxId);
  Serial.print("Found ");
  Serial.print(foundCount);
  Serial.println(" servo(s)");

  if (foundCount == 0)
  {
    Serial.println("No servos detected");
    Serial.println("Hint: Check wiring/power and the half-duplex circuit (XBUS is fixed at 250000 bps)");
    Serial.println();
    return;
  }

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

  applyFoundServos(foundServos, foundCount);

  Serial.print("Target ID auto-set to: ");
  Serial.println(targetServoID);
  Serial.print("Sync servo IDs auto-set to: ");
  printSyncServoIDs();
  Serial.println(" (Output: OFF)");
  Serial.println();
}

// [z]コマンド: ターゲットID変更
void executeChangeTargetIdCommand()
{
  Serial.print("Current target ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter new target ID (1-50): ");

  long newId;
  if (!parseNumber(readInput(), &newId) || !apiSetTarget(newId))
  {
    Serial.println("Error: ID out of range (1-50)");
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

  u8 newIDs[MAX_SYNC_SERVOS];
  int newCount = parseIdList(readInput(), newIDs);
  if (newCount < 0 || !apiSetSync(newIDs, newCount))
  {
    Serial.println("Error: Invalid ID list (valid IDs: 1-50)");
    return;
  }

  Serial.print("Sync servo IDs changed to: ");
  printSyncServoIDs();
  Serial.print(" (");
  Serial.print(syncServoCount);
  Serial.println(" servos, Output: OFF)");
}

// 現在位置と角度モードを読み取る（[r]とブラウザ用APIで共有）
XBusResult readServoPosition(u8 id, u16 *position, bool *angle180)
{
  long value;
  XBusResult result = xbusGet(id, ORDER_CURRENT_POS, &value);
  if (result != XBUS_OK)
    return result;

  *position = value & 0xFFFF;
  *angle180 = isAngle180(id);
  return XBUS_OK;
}

// [r]コマンド: サーボ位置読み取り
void executeReadPositionCommand()
{
  Serial.print("Reading position from servo ID ");
  Serial.println(targetServoID);

  u16 position;
  bool angle180;
  XBusResult result = readServoPosition(targetServoID, &position, &angle180);
  if (result != XBUS_OK)
  {
    Serial.print("Error: ");
    Serial.println(xbusResultText(result));
    return;
  }

  Serial.print("Position: 0x");
  Serial.print(position, HEX);
  Serial.print(" (");
  Serial.print(position);
  Serial.print(", ");
  Serial.print(positionToAngle(position, angle180), 1);
  Serial.print("°, ");
  Serial.print(angle180 ? "180" : "120");
  Serial.println("° mode)");
}

// [v]コマンド: サーボ負荷（CurrentPow）読み取り
void executeReadPowerCommand()
{
  Serial.print("Reading power from servo ID ");
  Serial.println(targetServoID);

  long value;
  XBusResult result = xbusGet(targetServoID, ORDER_CURRENT_POW, &value);
  if (result != XBUS_OK)
  {
    Serial.print("Error: ");
    Serial.println(xbusResultText(result));
    return;
  }

  Serial.print("Current power: ");
  Serial.print(value);
  Serial.println(" % (power applied to the motor, XBUS has no voltage order)");
}

// [a]コマンド: 全サーボ位置読み出し
void executeReadAllPositionsCommand()
{
  Serial.print("Reading positions from ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");
  Serial.println("ID\tPosition\tAngle(°)\tMode");
  Serial.println("-------------------------");

  u8 connectedCount = 0;

  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    long value;
    XBusResult result = xbusGet(id, ORDER_CURRENT_POS, &value);

    Serial.print(id);
    if (result == XBUS_OK)
    {
      u16 position = value & 0xFFFF;
      bool angle180 = isAngle180(id);
      Serial.print("\t0x");
      Serial.print(position, HEX);
      Serial.print("\t\t");
      Serial.print(positionToAngle(position, angle180), 1);
      Serial.println(angle180 ? "\t\t180°" : "\t\t120°");
      connectedCount++;
    }
    else
    {
      Serial.print("\t[Error] ");
      Serial.println(xbusResultText(result));
    }
  }

  Serial.println("-------------------------");
  Serial.print("Connected servos: ");
  Serial.print(connectedCount);
  Serial.print("/");
  Serial.println(syncServoCount);
}

// [c]コマンド: 全サーボ位置を中央にリセット（入力なし。ブラウザ用APIからも呼ぶ）
void executeResetAllPositionsCommand()
{
  Serial.print("Resetting ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(") to center (0x7FFF = 1500us)...");

  u16 positions[MAX_SYNC_SERVOS];
  for (u8 i = 0; i < syncServoCount; i++)
    positions[i] = XBUS_POSITION_NEUTRAL;
  moveSyncServos(positions);
  Serial.println("All servos are moving to center position (Output: ON)");
}

// [x]コマンド: 全サーボに角度を一斉送信（各サーボの角度モードで換算）
void executeAngleCommand()
{
  Serial.println("Enter angle (-90 to 90 degrees. 120° mode servos are limited to -60 to 60): ");

  String input = readInput();

  // 角度範囲チェック（範囲外ならapiMoveがfalse）
  if (input.length() == 0 || !apiMove(input.toFloat()))
    Serial.println("Error: Angle out of range (-90 to 90)");
}

// 一斉制御リストを角度へ（[x]の本体。範囲外ならfalse）
bool apiMove(float angle)
{
  if (angle < -90.0 || angle > 90.0)
    return false;

  // サーボごとに角度モードを確認して位置値に換算（900-2100usの範囲に制限）
  u16 positions[MAX_SYNC_SERVOS];
  for (u8 i = 0; i < syncServoCount; i++)
  {
    bool angle180 = isAngle180(syncServoIDs[i]);
    float limit = angle180 ? 90.0 : 60.0;
    float servoAngle = constrain(angle, -limit, limit);
    positions[i] = angleToPosition(servoAngle, angle180);

    Serial.print("ID ");
    Serial.print(syncServoIDs[i]);
    Serial.print(" (");
    Serial.print(angle180 ? "180" : "120");
    Serial.print("° mode): ");
    Serial.print(servoAngle, 1);
    Serial.print("deg -> 0x");
    Serial.print(positions[i], HEX);
    Serial.println(servoAngle != angle ? " [clamped]" : "");
  }

  moveSyncServos(positions);
  Serial.println("Angle command sent successfully (Output: ON)");
  return true;
}

// [e]コマンド: 出力ON（現在位置を読み取り、その位置で保持を開始。入力なし。ブラウザ用APIからも呼ぶ）
void executeOutputOnCommand()
{
  Serial.print("Output ON for ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");

  // 急に動かないよう、現在位置を目標にしてから送信開始
  u16 targets[MAX_SYNC_SERVOS];
  u8 readCount = 0;
  for (u8 i = 0; i < syncServoCount; i++)
  {
    long value;
    targets[i] = syncTargets[i];
    Serial.print("ID ");
    Serial.print(syncServoIDs[i]);
    if (xbusGet(syncServoIDs[i], ORDER_CURRENT_POS, &value) == XBUS_OK)
    {
      // 900-2100us外は脱力するため範囲内に制限
      targets[i] = constrain(value & 0xFFFF, XBUS_POSITION_MIN, XBUS_POSITION_MAX);
      readCount++;
      Serial.print(": hold at 0x");
      Serial.println(targets[i], HEX);
    }
    else
    {
      Serial.print(": No response, use target 0x");
      Serial.println(targets[i], HEX);
    }
  }

  lockBus();
  memcpy(syncTargets, targets, syncServoCount * sizeof(u16));
  outputEnabled = true;
  unlockBus();
  Serial.print("Output ON (channel data every ");
  Serial.print(XBUS_INTERVAL);
  Serial.print(" ms). Read current position: ");
  Serial.print(readCount);
  Serial.print("/");
  Serial.println(syncServoCount);
}

// [d]コマンド: 出力OFF（位置0を送って即座に脱力させてから送信停止。入力なし。ブラウザ用APIからも呼ぶ）
void executeOutputOffCommand()
{
  u16 positions[MAX_SYNC_SERVOS];
  for (u8 i = 0; i < syncServoCount; i++)
    positions[i] = XBUS_POSITION_RELEASE;
  moveSyncServos(positions);
  delay(XBUS_INTERVAL * 4); // 位置0を数回送信

  lockBus();
  outputEnabled = false;
  unlockBus();
  Serial.println("Output OFF (servos released by position 0)");
}

// オーダー一覧表示（writableOnly: 設定可能なものだけ）
void printOrderList(bool writableOnly)
{
  Serial.println("Order  Name          Size  Range / Access");
  for (u8 i = 0; i < XBUS_ORDER_COUNT; i++)
  {
    const XBusOrderInfo &info = XBUS_ORDERS[i];
    if (writableOnly && !info.writable)
      continue;

    char line[64];
    if (info.writable)
      snprintf(line, sizeof(line), "0x%02X   %-12s  %d     %ld to %ld", info.order, info.name, info.size, info.minValue, info.maxValue);
    else
      snprintf(line, sizeof(line), "0x%02X   %-12s  %d     read only", info.order, info.name, info.size);
    Serial.println(line);
  }
}

// [p]コマンド: 指定オーダーの値を取得
void executeReadRegisterCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  printOrderList(false);
  Serial.println("Enter order (e.g. 0x10 or 16): ");

  long order;
  if (!parseNumber(readInput(), &order) || order < 0 || order > 0xFF)
  {
    Serial.println("Error: Order out of range (0x00-0xFF)");
    return;
  }

  const XBusOrderInfo *info = findOrder(order);
  long value;
  XBusResult result = xbusGet(targetServoID, order, &value);
  if (result != XBUS_OK)
  {
    Serial.print("Error: ");
    Serial.println(xbusResultText(result));
    return;
  }

  Serial.print("Order 0x");
  Serial.print(order, HEX);
  Serial.print(" (");
  Serial.print(info ? info->name : "Unknown");
  Serial.print("): Dec=");
  Serial.print(value);
  if (orderSize(order) == 2)
  {
    Serial.print(" (signed ");
    Serial.print((short)value);
    Serial.print(")");
  }
  Serial.print(", Hex=0x");
  Serial.println(value & (orderSize(order) == 2 ? 0xFFFF : 0xFF), HEX);
}

// [s]コマンド: 指定オーダーに値を設定（読み返し確認・保存）
void executeWriteRegisterCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  printOrderList(true);
  Serial.println("Enter order (e.g. 0x10 or 16): ");

  long order;
  const XBusOrderInfo *info = NULL;
  if (parseNumber(readInput(), &order) && order >= 0 && order <= 0xFF)
    info = findOrder(order);
  if (info == NULL || !info->writable)
  {
    Serial.println("Error: Not a writable order (use [i] to change ID)");
    return;
  }

  Serial.print("Enter value (");
  Serial.print(info->minValue);
  Serial.print(" to ");
  Serial.print(info->maxValue);
  Serial.println("): ");

  long value;
  if (!parseNumber(readInput(), &value) || value < info->minValue || value > info->maxValue)
  {
    Serial.println("Error: Value out of range");
    return;
  }

  Serial.print("Setting ");
  Serial.print(info->name);
  Serial.print(" = ");
  Serial.print(value);
  Serial.print(" on servo ID ");
  Serial.println(targetServoID);

  XBusResult result = xbusSet(targetServoID, order, value);
  if (result != XBUS_OK)
  {
    Serial.print("Set failed: ");
    Serial.println(xbusResultText(result));
    return;
  }

  // 確認のため値を読み返す
  long readValue;
  result = xbusGet(targetServoID, order, &readValue);
  if (result != XBUS_OK)
  {
    Serial.print("Verification failed: ");
    Serial.println(xbusResultText(result));
    return;
  }

  long mask = (info->size == 2) ? 0xFFFF : 0xFF;
  bool match = (readValue & mask) == (value & mask);
  Serial.print("Verification: Read value = ");
  Serial.print(readValue);
  Serial.println(match ? " [OK]" : " [MISMATCH]");

  // 保存（ParamWrite）しないと電源OFFで元に戻る
  if (match)
  {
    Serial.println("Save to servo memory? (y = save, other = temporary): ");
    if (readInput() == "y")
    {
      result = xbusParamWrite(targetServoID, info->paramIndex);
      Serial.println(result == XBUS_OK ? "Saved [OK]" : "Save failed");
    }
    else
    {
      Serial.println("Not saved (will be lost at power off)");
    }
  }
}

// 受信なしでID変更（CH-ID 0x00で送信。接続中の全サーボが同じIDになるため1台だけで使う）
void executeChangeIdByBroadcast()
{
  Serial.println("Change ID by broadcast (CH-ID 0x00, TX only, no reply needed)?");
  Serial.println("[Warning] ALL connected servos get the same ID. Connect only ONE servo.");
  Serial.println("Enter new servo ID (1-50), or empty to cancel: ");

  long newId;
  String input = readInput();
  if (input.length() == 0)
  {
    Serial.println("Canceled");
    return;
  }
  if (!parseNumber(input, &newId) || newId < XBUS_ID_MIN || newId > XBUS_ID_MAX)
  {
    Serial.println("Error: ID out of range (1-50)");
    return;
  }

  // ID設定モード → 新ID（自動で通常モードに戻りROMに保存される）
  xbusSet(XBUS_BROADCAST_ID, ORDER_MODE, XBUS_MODE_ID_SET);
  delay(20);
  xbusSet(XBUS_BROADCAST_ID, ORDER_ID, newId);
  delay(100);

  u8 ids[1] = {(u8)newId};
  setSyncServoList(ids, 1);
  targetServoID = newId;

  // 受信できないため、新IDだけを動かして目視で確認
  Serial.print("ID set to ");
  Serial.print(newId);
  Serial.println(" by broadcast. Sweeping the new ID for 2 s to confirm...");
  sweepIdRange(newId, newId, 2000, false);
  Serial.println("If the servo moved, the ID change succeeded. Target ID and sync servo IDs are set to it.");
}

// IDを変更する（[i]の本体。旧IDの応答確認は呼び出し側で行う）。新IDで応答すればtrue
bool changeServoId(u8 oldId, long newId)
{
  if (newId < XBUS_ID_MIN || newId > XBUS_ID_MAX)
  {
    Serial.println("Error: ID out of range (1-50)");
    return false;
  }

  if (newId == oldId)
  {
    Serial.println("New ID is the same as current ID. Nothing to do.");
    return false;
  }

  // ID重複チェック
  if (xbusPing(newId, NULL, SCAN_TIMEOUT) != XBUS_NO_RESPONSE)
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

  // ID設定モードに切り替え → 新ID書き込み（JR PROPO XBusServoライブラリと同じ手順）
  XBusResult result = xbusSet(oldId, ORDER_MODE, XBUS_MODE_ID_SET);
  if (result != XBUS_OK)
  {
    Serial.print("Error: Failed to enter ID set mode (");
    Serial.print(xbusResultText(result));
    Serial.println(")");
    return false;
  }
  xbusSet(oldId, ORDER_ID, newId); // 応答IDが新旧どちらでも最後にPINGで確認
  delay(50);

  // 新IDで応答確認
  if (xbusPing(newId, NULL, RESPONSE_TIMEOUT) != XBUS_OK)
  {
    Serial.println("Verification failed: No response at new ID");
    Serial.println("Hint: Use [f] to scan servos");
    return false;
  }

  // 一斉制御リスト内の旧IDも新IDに置き換え（昇順を維持）
  u8 newList[MAX_SYNC_SERVOS];
  u8 newCount = 0;
  bool inserted = false;
  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    if (id == oldId)
    {
      inserted = true;
      continue;
    }
    newList[newCount++] = id;
  }
  if (inserted)
  {
    u8 pos = newCount;
    while (pos > 0 && newList[pos - 1] > newId)
    {
      newList[pos] = newList[pos - 1];
      pos--;
    }
    newList[pos] = newId;
    newCount++;
    setSyncServoList(newList, newCount);
  }

  targetServoID = newId;
  Serial.print("Servo ID changed successfully [OK]. Target ID is now ");
  Serial.println(targetServoID);
  return true;
}

// [i]コマンド: サーボID変更
void executeChangeServoIdCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);

  // 変更元サーボの存在確認（応答がなければ受信なしのブロードキャスト変更を提案）
  if (xbusPing(targetServoID, NULL, RESPONSE_TIMEOUT) != XBUS_OK)
  {
    Serial.println("No response from target servo.");
    executeChangeIdByBroadcast();
    return;
  }

  Serial.println("Enter new servo ID (1-50): ");

  long newId;
  if (!parseNumber(readInput(), &newId))
  {
    Serial.println("Error: ID out of range (1-50)");
    return;
  }

  changeServoId(targetServoID, newId);
}

// [m]コマンド: サーボのバージョン情報（型番/バージョン）読み取り
void executeReadVersionCommand()
{
  Serial.print("Reading version info from servo ID ");
  Serial.println(targetServoID);

  long product, version;
  XBusResult result = xbusGet(targetServoID, ORDER_PRODUCT, &product);
  if (result == XBUS_OK)
    result = xbusGet(targetServoID, ORDER_VERSION, &version);
  if (result != XBUS_OK)
  {
    Serial.print("Error: ");
    Serial.println(xbusResultText(result));
    return;
  }

  Serial.println("Protocol        : JR XBUS");
  Serial.print("Product         : 0x");
  Serial.print(product & 0xFFFF, HEX);
  Serial.print(" -> ");
  Serial.println(modelName(product & 0xFFFF));
  Serial.print("Version         : 0x");
  Serial.println(version & 0xFFFF, HEX);

  long reverse;
  if (xbusGet(targetServoID, ORDER_REVERSE, &reverse) == XBUS_OK)
  {
    Serial.print("Direction       : ");
    Serial.println(reverse ? "Reversed" : "Normal");
  }

  long angle180;
  if (xbusGet(targetServoID, ORDER_ANGLE_180, &angle180) == XBUS_OK)
  {
    Serial.print("Angle mode      : ");
    Serial.println(angle180 == 1 ? "180° (900-2100us = ±90°)" : "120° (900-2100us = ±60°)");
  }
}

// [o]コマンド: 回転方向設定（Reverse, 保存あり）
void executeSetDirectionCommand()
{
  long current;
  XBusResult result = xbusGet(targetServoID, ORDER_REVERSE, &current);
  if (result != XBUS_OK)
  {
    Serial.print("Error: ");
    Serial.println(xbusResultText(result));
    return;
  }

  Serial.print("Servo ID ");
  Serial.print(targetServoID);
  Serial.print(" direction: ");
  Serial.println(current ? "Reversed" : "Normal");
  Serial.println("Select direction: [0] Normal  [1] Reversed");

  String input = readInput();
  if (input != "0" && input != "1")
  {
    Serial.println("Error: Invalid selection");
    return;
  }

  long value = input.toInt();
  if (xbusSet(targetServoID, ORDER_REVERSE, value) != XBUS_OK ||
      xbusParamWrite(targetServoID, PARAM_IDX_REVERSED) != XBUS_OK)
  {
    Serial.println("Error: Failed to set direction");
    return;
  }

  long readValue;
  if (xbusGet(targetServoID, ORDER_REVERSE, &readValue) == XBUS_OK)
  {
    Serial.print("Verification: Read value = ");
    Serial.print(readValue);
    Serial.println(readValue == value ? " [OK] (saved)" : " [MISMATCH]");
  }
}

// [q]コマンド: 工場出荷リセット（ParamReset 0x0001をCH-ID 0x00へ。全パラメータとIDを初期化し自動保存）
void executeFactoryResetCommand()
{
  Serial.println("[Warning] Factory reset is sent to ALL connected servos (CH-ID 0x00).");
  Serial.println("          All parameters are reset and ID becomes 1.");
  Serial.println("          Connect only ONE servo. Type YES to continue: ");

  if (readInput() != "YES")
  {
    Serial.println("Canceled");
    return;
  }

  // 出力を止めてからリセット（ブロードキャストは応答なし）
  lockBus();
  outputEnabled = false;
  unlockBus();

  Serial.println("Executing factory reset...");
  xbusSet(XBUS_BROADCAST_ID, ORDER_PARAM_RESET, PARAM_IDX_ALL_WITH_ID);
  delay(500);

  targetServoID = 1;
  u16 modelNumber;
  if (xbusPing(targetServoID, &modelNumber, RESPONSE_TIMEOUT) == XBUS_OK)
  {
    Serial.print("Factory reset completed [OK]. Found ID 1 (");
    Serial.print(modelName(modelNumber));
    Serial.println(")");
  }
  else
  {
    Serial.println("Verification failed: No response at ID 1");
    Serial.println("Hint: Use [f] to scan servos");
  }
}

// 指定ID範囲の全員に同じ位置指令を送る（受信不要。呼び出し側でバスを確保すること）
void sendPositionToIdRange(u8 firstId, u8 lastId, u16 position)
{
  static u8 packet[MAX_PACKET_SIZE];
  u16 index = 0;
  packet[index++] = XBUS_CMD_MODE_A;
  packet[index++] = (lastId - firstId + 1) * 4 + 2;
  packet[index++] = 0x00;
  packet[index++] = 0x00;
  for (u8 id = firstId; id <= lastId; id++)
  {
    packet[index++] = id;
    packet[index++] = 0x00;
    packet[index++] = position >> 8;
    packet[index++] = position & 0xFF;
  }
  packet[index] = crc8(packet, index);
  index++;
  sendPacket(packet, index);
}

// 指定ID範囲を一定時間往復させ、最後に脱力させる（受信不要）。Enter入力で中断した場合true
bool sweepIdRange(u8 firstId, u8 lastId, unsigned long duration, bool stopOnInput)
{
  bool interrupted = false;
  lockBus();
  outputEnabled = false;

  unsigned long startTime = millis();
  unsigned long lastSend = 0;
  while (millis() - startTime < duration)
  {
    if (stopOnInput && Serial.available())
    {
      interrupted = true;
      break;
    }
    if (millis() - lastSend < XBUS_INTERVAL)
      continue;
    lastSend = millis();

    unsigned long elapsed = millis() - startTime;
    sendPositionToIdRange(firstId, lastId, angleToPosition(30.0 * sin(2.0 * PI * elapsed / 1000.0), false));
  }

  // 位置0を数回送って脱力
  for (u8 i = 0; i < 4; i++)
  {
    sendPositionToIdRange(firstId, lastId, XBUS_POSITION_RELEASE);
    delay(XBUS_INTERVAL);
  }
  unlockBus();
  return interrupted;
}

// [t]コマンド: 送信テスト（ID1-50全員に位置指令を送り、約5秒往復。受信不要でTX側の確認用。ブラウザ用APIからも呼ぶ）
void executeTxTestCommand()
{
  Serial.println("TX test: sending channel data to ALL IDs (1-50) for 5 s (sweep +/-30 deg).");
  Serial.println("No reception is needed. If the servo moves, TX (ESP32 -> servo) is OK.");

  sweepIdRange(XBUS_ID_MIN, XBUS_ID_MAX, 5000, false);

  Serial.println("TX test finished (servo released).");
  Serial.println("Moved     -> TX OK. If [f] finds nothing, check RX path (use [g]), and use [w] to find the ID.");
  Serial.println("Not moved -> check power, signal wiring/level, and power-cycle the servo");
  Serial.println("             (it locks to PWM if it received PWM first after power on).");
}

// [w]コマンド: ID探索（受信不要。IDを1つずつ動かし、動いたところでEnterを押してもらう）
void executeFindIdByMotionCommand()
{
  Serial.println("Find ID by motion (TX only): each ID 1-50 is swept for 1.5 s in turn.");
  Serial.println("Press Enter as soon as the servo moves.");
  delay(1000);
  flushReceiveBuffer();
  while (Serial.available())
    Serial.read();

  for (u8 id = XBUS_ID_MIN; id <= XBUS_ID_MAX; id++)
  {
    Serial.print("Sweeping ID ");
    Serial.println(id);
    if (sweepIdRange(id, id, 1500, true))
    {
      while (Serial.available())
        Serial.read();

      // 反応までの遅れを考慮し、直前のIDも候補として表示
      Serial.print("Stopped at ID ");
      Serial.print(id);
      if (id > XBUS_ID_MIN)
      {
        Serial.print(" (if you pressed late, it may be ID ");
        Serial.print(id - 1);
        Serial.print(")");
      }
      Serial.println();

      u8 ids[1] = {id};
      setSyncServoList(ids, 1);
      targetServoID = id;
      Serial.print("Target ID and sync servo IDs set to: ");
      Serial.println(id);
      Serial.println("Check with [x]. If another servo moves, set the ID with [z] / [n].");
      return;
    }
  }
  Serial.println("No input. The servo did not move at any ID (1-50)?");
}

// 1つのIDにGet Productを送り、windowMs間に受信した生データを集める。受信数を返す
u8 rxDumpOne(u8 id, u8 *packet, u8 *sentLength, u8 *received, u8 maxCount, unsigned long *firstAt, unsigned long windowMs)
{
  *sentLength = buildCommandPacket(packet, XBUS_CMD_GET, id, ORDER_PRODUCT, 0, 2);
  u8 count = 0;

  lockBus();
  sendPacketToRx(packet, *sentLength); // エコーも表示するため読み捨てない
  unsigned long startTime = micros();
  while (micros() - startTime < windowMs * 1000 && count < maxCount)
  {
    if (servoSerial->available())
    {
      if (count == 0)
        *firstAt = micros() - startTime;
      received[count++] = servoSerial->read();
    }
  }
  busDirTx();
  unlockBus();
  return count;
}

// RX端子の波形記録の結果
struct RxTrace
{
  uint32_t lowUs;     // LOWだった時間の合計(us)
  uint32_t firstLowUs; // 最初にLOWになった時刻(us, 送信完了から)
  uint32_t lastLowUs;  // 最後にLOWだった時刻(us)
  u16 edges;     // 変化の回数
};

// RX端子の波形を直接記録（UARTを通さない）。サーボが線を駆動しているかを確認する
static void __attribute__((noinline)) IRAM_ATTR traceRxPin(int8_t pinRx, uint32_t durationUs, RxTrace *trace)
{
  const uint32_t cyclesPerUs = 240;
  uint32_t start = xthal_get_ccount();
  uint32_t last = start;
  int prev = gpio_ll_get_level(&GPIO, (gpio_num_t)pinRx);
  uint32_t lowCycles = 0;
  trace->edges = 0;
  trace->firstLowUs = 0xFFFFFFFF;
  trace->lastLowUs = 0;

  portENTER_CRITICAL(&busMux);
  while (true)
  {
    uint32_t now = xthal_get_ccount();
    uint32_t elapsed = now - start;
    if (elapsed > durationUs * cyclesPerUs)
      break;
    int level = gpio_ll_get_level(&GPIO, (gpio_num_t)pinRx);
    if (!level)
    {
      lowCycles += now - last;
      if (trace->firstLowUs == 0xFFFFFFFF)
        trace->firstLowUs = elapsed / cyclesPerUs;
      trace->lastLowUs = elapsed / cyclesPerUs;
    }
    if (level != prev)
      trace->edges++;
    prev = level;
    last = now;
  }
  portEXIT_CRITICAL(&busMux);
  trace->lowUs = lowCycles / cyclesPerUs;
}

// 1つのIDにGet Productを送り、受信方向でRX端子の波形を記録する
void traceReplyLine(u8 id, RxTrace *trace)
{
  u8 packet[8];
  u8 length = buildCommandPacket(packet, XBUS_CMD_GET, id, ORDER_PRODUCT, 0, 2);
  lockBus();
  sendPacketToRx(packet, length);
  traceRxPin(RX1, 3000, trace);
  busDirTx();
  unlockBus();
}

// 波形記録の結果を表示
void printTrace(const RxTrace &trace)
{
  if (trace.edges == 0 && trace.lowUs == 0)
  {
    Serial.println("line stayed HIGH (no activity)");
    return;
  }
  Serial.print("LOW for ");
  Serial.print(trace.lowUs);
  Serial.print(" us, ");
  Serial.print(trace.edges);
  Serial.print(" edges, from ");
  Serial.print(trace.firstLowUs);
  Serial.print(" to ");
  Serial.print(trace.lastLowUs);
  Serial.println(" us after TX end");
}

// 受信方向で待機したときのRX端子のレベル（XBUSのアイドルはHIGH）
void printRxIdleLevel()
{
  lockBus();
  busDirRx();
  delayMicroseconds(200);
  int highCount = 0;
  for (int i = 0; i < 100; i++)
  {
    highCount += digitalRead(RX1) ? 1 : 0;
    delayMicroseconds(10);
  }
  busDirTx();
  unlockBus();

  Serial.print("RX pin level while listening: ");
  if (highCount >= 95)
    Serial.println("HIGH (OK: idle level is seen through the RX path)");
  else if (highCount <= 5)
    Serial.println("LOW (NG: RX path does not pass the idle HIGH. Check buffer/level shifter and pull-up)");
  else
  {
    Serial.print("unstable (");
    Serial.print(highCount);
    Serial.println("% HIGH: line may be floating. Add a pull-up on the signal line)");
  }
}

// [g]コマンド: 受信テスト（Get Productを送り、受信した生データを16進表示。allでID1-50を調べる）
void executeRxDumpCommand()
{
  Serial.println("Enter ID to test (empty = target ID, 'all' = ID1-50): ");
  String input = readInput();

  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());
  printRxIdleLevel();

  u8 packet[8], sentLength, received[64];
  unsigned long firstAt = 0;

  if (input == "all")
  {
    // 位置指令はサーボIDだけで受け取るが、Get/Setはサブ ID（上位2ビット）まで一致しないと返信しないため全サブ ID を試す
    Serial.println("Sending Get Product to ID1-50 x sub-ID 0-3 (20 ms window each)...");
    u8 idsWithData = 0;
    for (u16 n = 0; n < 4 * XBUS_ID_MAX; n++)
    {
      u8 sub = n / XBUS_ID_MAX;
      u8 id = (n % XBUS_ID_MAX) + XBUS_ID_MIN;
      u8 channel = id | (sub << 6);
      u8 count = rxDumpOne(channel, packet, &sentLength, received, sizeof(received), &firstAt, 20);
      bool echoOnly = (count == sentLength && memcmp(received, packet, sentLength) == 0);
      if (count == 0 || echoOnly)
        continue;
      idsWithData++;
      Serial.print("  ID ");
      Serial.print(id);
      Serial.print(" sub-ID ");
      Serial.print(sub);
      Serial.print(": ");
      for (u8 k = 0; k < count; k++)
      {
        if (received[k] < 0x10)
          Serial.print("0");
        Serial.print(received[k], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    if (idsWithData == 0)
      Serial.println("-> No UART data from any ID.");
    else
      Serial.println("-> Some data received. If it is not '22 ...', the signal level / timing is marginal.");

    // UARTを通さずにRX端子の波形を確認（サーボが線を駆動しているか）
    Serial.println("Tracing RX pin directly for 3 ms after each request (ID1-50 x sub-ID 0-3)...");
    u8 idsWithActivity = 0;
    for (u16 n = 0; n < 4 * XBUS_ID_MAX; n++)
    {
      u8 sub = n / XBUS_ID_MAX;
      u8 id = (n % XBUS_ID_MAX) + XBUS_ID_MIN;
      RxTrace trace;
      traceReplyLine(id | (sub << 6), &trace);
      if (trace.edges == 0 && trace.lowUs == 0)
        continue;
      idsWithActivity++;
      Serial.print("  ID ");
      Serial.print(id);
      Serial.print(" sub-ID ");
      Serial.print(sub);
      Serial.print(": ");
      printTrace(trace);
      delay(2);
    }
    if (idsWithActivity == 0)
      Serial.println("-> RX line stayed HIGH for all IDs and sub-IDs: the servo does not reply to commands "
                     "(model without command support?), or the RX path does not pass the servo signal.");
    else if (idsWithData == 0)
      Serial.println("-> The servo drives the line, but the UART cannot decode it: signal level problem "
                     "(3.3V reply is not seen as HIGH by the receive buffer). See README.");
    else
      Serial.println("-> The servo replies.");
    return;
  }

  long id = targetServoID;
  if (input.length() > 0 && (!parseNumber(input, &id) || id < XBUS_ID_MIN || id > XBUS_ID_MAX))
  {
    Serial.println("Error: ID out of range (1-50)");
    return;
  }

  u8 count = rxDumpOne(id, packet, &sentLength, received, sizeof(received), &firstAt, 50);

  Serial.print("Sent to ID ");
  Serial.print(id);
  Serial.print(": ");
  for (u8 k = 0; k < sentLength; k++)
  {
    if (packet[k] < 0x10)
      Serial.print("0");
    Serial.print(packet[k], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Received ");
  Serial.print(count);
  Serial.print(" byte(s) in 50 ms: ");
  for (u8 k = 0; k < count; k++)
  {
    if (received[k] < 0x10)
      Serial.print("0");
    Serial.print(received[k], HEX);
    Serial.print(" ");
  }
  Serial.println();
  if (count > 0)
  {
    Serial.print("First byte at ");
    Serial.print(firstAt);
    Serial.println(" us after TX end");
  }

  RxTrace trace;
  traceReplyLine(id, &trace);
  Serial.print("RX pin trace (3 ms): ");
  printTrace(trace);

  // 結果の解釈
  if (count == 0)
    Serial.println("-> Nothing received. Wrong ID, or RX path does not see the servo's 3.3V reply. Try 'all'.");
  else if (count == sentLength && memcmp(received, packet, sentLength) == 0)
    Serial.println("-> Only the echo of our own packet. Servo did not reply (wrong ID? try 'all').");
  else if (received[0] == XBUS_CMD_STATUS || (count > sentLength && received[sentLength] == XBUS_CMD_STATUS))
    Serial.println("-> Status reply detected. Communication looks OK.");
  else
    Serial.println("-> Unexpected data. Possible signal level / noise problem.");
}

void disp_menu()
{
  Serial.println("============================================");
  Serial.print("Current target ID: ");
  Serial.print(targetServoID);
  Serial.print(", Sync IDs: ");
  printSyncServoIDs();
  Serial.print(", Output: ");
  Serial.print(outputEnabled ? "ON" : "OFF");
  Serial.print(", Baud rate: ");
  Serial.print(XBUS_BAUD_RATE);
  Serial.println(" bps (fixed), Protocol: JR XBUS");
  Serial.println("Menu: [f] Find servos   [n] Set sync servo IDs   [z] Set target ID ");
  Serial.println("      [v] Read power    [r] Read position        [p] Read order");
  Serial.println("      [c] All to center [a] Read all positions   [s] Write order");
  Serial.println("      [e] Output ON     [d] Output OFF           [x] Send angle to all ");
  Serial.println("      [i] Change ID     [m] Read version         [o] Set direction");
  Serial.println("      [q] Factory reset [t] TX test (all IDs)    [g] RX dump (debug)");
  Serial.println("      [w] Find ID by motion (TX only)");
  Serial.println("      [h] Change servo type (restart)");
  Serial.println("Input Command:");
}

// 初期化
void setup()
{
  Serial.begin(115200);
  // サーボバス開始（250000bps 8N1, エコー自動判定, 送信モードで待機）
  busBegin(&Serial1, RX1, TX1, PIN_EN, XBUS_BAUD_RATE, SERIAL_8N1);

  // 一斉制御の目標を中央に初期化
  for (u8 i = 0; i < MAX_SYNC_SERVOS; i++)
    syncTargets[i] = XBUS_POSITION_NEUTRAL;

  // バス排他とチャンネルデータ送信タスク（loopと別コアで動作）
  if (busMutex == NULL) // 自動判別で作成済みのことがある
    busMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(channelOutputTask, "xbusOutput", 4096, NULL, 2, NULL, 0);

  delay(1000);

  Serial.println("=====================================");
  Serial.println("JR XBUS Servo Setter");
  Serial.println("XBUS servo (250000 bps, 8N1)");
  Serial.println("=====================================");
  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());
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
      executeReadPowerCommand();
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
      executeOutputOnCommand();
    }
    else if (command == "d")
    {
      executeOutputOffCommand();
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
    else if (command == "o")
    {
      executeSetDirectionCommand();
    }
    else if (command == "q")
    {
      executeFactoryResetCommand();
    }
    else if (command == "t")
    {
      executeTxTestCommand();
    }
    else if (command == "g")
    {
      executeRxDumpCommand();
    }
    else if (command == "w")
    {
      executeFindIdByMotionCommand();
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

// 読み取りコマンド（型番の取得）だけで XBUS のサーボを探す（チャンネルデータは送らないのでサーボは動かない）
bool detect()
{
  busBegin(servoSerial, RX1, TX1, PIN_EN, XBUS_BAUD_RATE, SERIAL_8N1);
  if (busMutex == NULL)
    busMutex = xSemaphoreCreateMutex();
  for (int id = XBUS_ID_MIN; id <= DEFAULT_SCAN_MAX_ID; id++)
  {
    if (xbusPing(id, NULL, SCAN_TIMEOUT) == XBUS_OK)
      return true;
  }
  return false;
}

// ===== ブラウザ用API =====

// 今の状態と、XBUSでできること
void apiGetInfo(ApiInfo *info)
{
  info->idMin = XBUS_ID_MIN;
  info->idMax = XBUS_ID_MAX;
  info->angleMin = -90.0; // [x]と同じ（120度モードのサーボは±60度に制限される）
  info->angleMax = 90.0;
  info->bauds = NULL; // 250000bps固定
  info->baudCount = 0;
  info->baudIndex = -1;
  info->baud = XBUS_BAUD_RATE;
  info->target = targetServoID;
  info->syncIds = syncServoIDs;
  info->syncCount = syncServoCount;
  info->oneServoForId = false; // アドレス指定でID変更する（ブロードキャスト変更は[i]の対話だけ）
}

// [f]と同じ検索（ID1-maxId。50を超える指定は50まで）。見つかればターゲットとリストも設定
int apiScan(int maxId, ApiServo *found, int maxCount)
{
  static u8 foundServos[MAX_SYNC_SERVOS];
  static u16 foundModels[MAX_SYNC_SERVOS];

  if (maxId < XBUS_ID_MIN)
    return -1;
  if (maxId > XBUS_ID_MAX)
    maxId = XBUS_ID_MAX;

  Serial.print("Scanning servos from ID1 to ID");
  Serial.print(maxId);
  Serial.print(" at ");
  Serial.print(XBUS_BAUD_RATE);
  Serial.println(" bps...");

  u8 foundCount = scanServos(maxId, foundServos, foundModels);
  if (foundCount > 0)
    applyFoundServos(foundServos, foundCount);

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

// ターゲットID変更（[z]と同じ範囲チェック）
bool apiSetTarget(int id)
{
  if (id < XBUS_ID_MIN || id > XBUS_ID_MAX)
    return false;
  targetServoID = id;
  return true;
}

// 一斉制御リスト設定（[n]と同じ。昇順・重複なしに並べ替え、出力OFF・目標は中央）
bool apiSetSync(const uint8_t *ids, uint8_t count)
{
  if (count == 0)
    return false;

  bool selected[XBUS_ID_MAX + 1] = {false};
  for (u8 i = 0; i < count; i++)
  {
    if (ids[i] < XBUS_ID_MIN || ids[i] > XBUS_ID_MAX)
      return false;
    selected[ids[i]] = true;
  }

  u8 list[MAX_SYNC_SERVOS];
  u8 listCount = 0;
  for (int id = XBUS_ID_MIN; id <= XBUS_ID_MAX; id++)
  {
    if (selected[id])
      list[listCount++] = id;
  }
  setSyncServoList(list, listCount);
  return true;
}

// 一斉制御リストを中央へ（[c]と同じ）
bool apiCenter()
{
  executeResetAllPositionsCommand();
  return true;
}

// ON: [e]と同じ（現在位置で保持）, OFF: [d]と同じ（位置0で脱力して送信停止）
bool apiTorque(bool on)
{
  if (on)
    executeOutputOnCommand();
  else
    executeOutputOffCommand();
  return true;
}

// 送信テスト（[t]と同じ）
bool apiTxTest()
{
  executeTxTestCommand();
  return true;
}

// 現在位置を角度で読む（[r]と同じ換算。180度モードも考慮）
bool apiReadPosition(int id, float *deg)
{
  if (id < XBUS_ID_MIN || id > XBUS_ID_MAX)
    return false;

  u16 position;
  bool angle180;
  if (readServoPosition(id, &position, &angle180) != XBUS_OK)
    return false;

  *deg = positionToAngle(position, angle180);
  return true;
}

// ID変更（[i]のアドレス指定の手順。旧IDが応答しなければfalse）
bool apiSetId(int oldId, int newId)
{
  if (oldId < XBUS_ID_MIN || oldId > XBUS_ID_MAX)
    return false;

  if (xbusPing(oldId, NULL, RESPONSE_TIMEOUT) != XBUS_OK)
  {
    Serial.println("No response from target servo.");
    return false;
  }

  return changeServoId(oldId, newId);
}

// XBUSは250000bps固定のため変更できない
bool apiSetBaud(int index, bool servoToo)
{
  (void)index;
  (void)servoToo;
  return false;
}

} // namespace jrxbus
