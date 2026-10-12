#include <Arduino.h>
#include "servo_bus.h"
#include "servo_select.h"

namespace kondo
{

// 基本型定義
typedef unsigned char u8;
typedef unsigned short u16;

// ピン定義（Meridian Board L系統と同じ）
#define PIN_EN 33 // 半二重通信の送受信切替（HIGH: 送信, LOW: 受信）

// ICS通信定義（ICS3.5/3.6 ソフトウェアマニュアル 第2.2版より）
#define ICS_SERIAL_CONFIG SERIAL_8E1 // 8bit, 偶数パリティ, ストップ1bit

// ICSコマンド定義
#define ICS_CMD_POSITION 0x80 // ポジション設定（0x80 + ID, POS_H, POS_L）
#define ICS_CMD_READ 0xA0     // パラメータ読み出し（0xA0 + ID, SC）
#define ICS_CMD_WRITE 0xC0    // パラメータ書き込み（0xC0 + ID, SC, DATA）
#define ICS_CMD_ID 0xE0       // ID書き込み（0xE0 + ID, 1, 1, 1）
#define ICS_CMD_ID_READ 0xFF  // ID読み出し（0xFF, 0, 0, 0）

// サブコマンド定義
#define ICS_SC_EEPROM 0x00   // EEPROM（64バイト）
#define ICS_SC_STRETCH 0x01  // ストレッチ 1-127
#define ICS_SC_SPEED 0x02    // スピード 1-127
#define ICS_SC_CURRENT 0x03  // 電流（読み: 0-63正転/64-127逆転, 書き: 制限値1-63）
#define ICS_SC_TEMP 0x04     // 温度（読み: 小さいほど高温, 書き: 制限値1-127）
#define ICS_SC_POSITION 0x05 // 現在位置（ICS3.6のみ, 読み出し専用）

// ID・位置定義
#define ICS_ID_MIN 0
#define ICS_ID_MAX 31
#define ICS_POS_MIN 3500
#define ICS_POS_CENTER 7500
#define ICS_POS_MAX 11500
#define ICS_POS_FREE 0           // 位置0で脱力（返信で現在位置を取得）
#define ICS_COUNT_PER_DEG 29.633 // 近藤科学 IcsClass の換算値（270度で8000カウント）
#define ICS_ANGLE_LIMIT 135.0

// EEPROM定義（64バイト。1パラメータ = 上位4bit・下位4bitの2バイト）
#define EEPROM_SIZE 64
#define EEPROM_BACKUP_CHAR 0x5A // 先頭の固定値（書き換え禁止）
#define EE_STRETCH 2            // オフセット（マニュアルのBYTE番号 - 1）
#define EE_SPEED 4
#define EE_PUNCH 6
#define EE_DEAD_BAND 8
#define EE_DAMPING 10
#define EE_SAFE_TIMER 12
#define EE_FLAGS 14
#define EE_LIMIT_UPPER 16 // 2バイト値（4ニブル）
#define EE_LIMIT_LOWER 20 // 2バイト値（4ニブル）
#define EE_BAUD 26
#define EE_TEMP_LIMIT 28
#define EE_CURRENT_LIMIT 30
#define EE_RESPONSE 50
#define EE_USER_OFFSET 52
#define EE_ID 56
#define EE_CHAR_STRETCH1 58
#define EE_CHAR_STRETCH2 60
#define EE_CHAR_STRETCH3 62

// フラグのビット（組み立てた8bit値）
#define FLAG_SLAVE 0x80
#define FLAG_ROTATION 0x10
#define FLAG_PWMINH 0x08
#define FLAG_FIXED_ONE 0x04 // 常に1（書き換え禁止）
#define FLAG_FREE 0x02      // 読み出し専用
#define FLAG_REVERSE 0x01

// 通信速度（インデックス0: 115200, 1: 625000, 2: 1250000）
const unsigned long ICS_BAUD_RATES[] = {115200, 625000, 1250000};
#define ICS_BAUD_COUNT (sizeof(ICS_BAUD_RATES) / sizeof(ICS_BAUD_RATES[0]))
#define DEFAULT_BAUD_INDEX 0
// EEPROMの通信速度値: 625000 = 1, 1250000 = 0。115200はマニュアルに「10」「0x10」両方の記載があるため実機値で判定
#define EE_BAUD_625K 0x01
#define EE_BAUD_1250K 0x00
#define EE_BAUD_115K_DEC 0x0A
#define EE_BAUD_115K_HEX 0x10

// 通信タイムアウト定義
#define RESPONSE_TIMEOUT 50 // 通常の応答待ち(ms)
#define SCAN_TIMEOUT 10     // サーボ検索時の応答待ち(ms)
#define ID_CMD_TIMEOUT 1000 // IDコマンドの応答待ち(ms)
#define ID_CMD_WAIT 520     // IDコマンド後、次のコマンドを受け付けるまでの待ち(ms, 近藤科学IcsClassと同じ)

// 一斉制御定義
#define MAX_SYNC_SERVOS 32

// グローバル変数
HardwareSerial *servoSerial = &Serial1;                 // 使用するシリアルポート
u8 targetServoID = 0;                                   // ターゲットサーボID
u8 syncServoIDs[MAX_SYNC_SERVOS] = {0};                 // 一斉制御するサーボIDリスト（とびとびIDも可）
u8 syncServoCount = 1;                                  // リスト内のサーボ数
u8 servoBaudIndex = DEFAULT_BAUD_INDEX;                 // ESP32側の現在の通信速度インデックス

// ===== 半二重UART（servo_bus.h） =====

// 受信バッファフラッシュ
void flushReceiveBuffer()
{
  busFlushRx();
}

// 送受信（送信 → すぐ受信に切替 → エコー照合して読み捨て → 応答rxLenバイトを受信）
bool icsTransfer(const u8 *tx, u8 txLen, u8 *rx, u8 rxLen, unsigned long timeoutMs)
{
  busFlushRx();
  busDirTx();
  servoSerial->write(tx, txLen);
  busEndSendToRx();

  if (!busSkipEcho(tx, txLen, timeoutMs))
    return false;

  unsigned long startTime = millis();
  u8 rxPos = 0;
  while (rxPos < rxLen)
  {
    if (servoSerial->available() > 0)
      rx[rxPos++] = servoSerial->read();
    else if (millis() - startTime >= timeoutMs)
      return false;
  }
  return true;
}

// シリアル開始（8E1）とエコー判定。ICSは送信しないときは常に受信方向（マニュアル p.9）
void beginServoBus()
{
  busBegin(servoSerial, RX1, TX1, PIN_EN, ICS_BAUD_RATES[servoBaudIndex], ICS_SERIAL_CONFIG, false, false);
}

// ESP32側の通信速度を変更（ピン設定・エコー設定は維持）
void setHostBaudRate(u8 baudIndex)
{
  servoSerial->flush();
  servoSerial->updateBaudRate(ICS_BAUD_RATES[baudIndex]);
  servoBaudIndex = baudIndex;
  delay(10);
  flushReceiveBuffer();
}

// ===== ICS コマンド =====

// ポジション設定。成功時は返信の現在位置（失敗時 -1）。pos = 0 で脱力
int icsSetPosition(u8 id, u16 pos, unsigned long timeoutMs = RESPONSE_TIMEOUT)
{
  u8 tx[3] = {(u8)(ICS_CMD_POSITION | id), (u8)((pos >> 7) & 0x7F), (u8)(pos & 0x7F)};
  u8 rx[3];
  if (!icsTransfer(tx, 3, rx, 3, timeoutMs))
    return -1;
  // 返信は ID（ID0かつ115200bpsのときのみMSBが1になる）
  if ((rx[0] & 0x1F) != id || (rx[0] & 0x60) != 0)
    return -1;
  return ((rx[1] & 0x7F) << 7) | (rx[2] & 0x7F);
}

// パラメータ読み出し（SC 0x01-0x04: 1バイト）。失敗時 -1
int icsReadParam(u8 id, u8 sc, unsigned long timeoutMs = RESPONSE_TIMEOUT)
{
  u8 tx[2] = {(u8)(ICS_CMD_READ | id), sc};
  u8 rx[3];
  if (!icsTransfer(tx, 2, rx, 3, timeoutMs))
    return -1;
  if (rx[0] != (0x20 | id) || rx[1] != sc)
    return -1;
  return rx[2];
}

// 現在位置の読み出し（SC 0x05, ICS3.6のみ）。失敗時 -1
int icsReadPosition(u8 id, unsigned long timeoutMs = RESPONSE_TIMEOUT)
{
  u8 tx[2] = {(u8)(ICS_CMD_READ | id), ICS_SC_POSITION};
  u8 rx[4];
  if (!icsTransfer(tx, 2, rx, 4, timeoutMs))
    return -1;
  if (rx[0] != (0x20 | id) || rx[1] != ICS_SC_POSITION)
    return -1;
  return ((rx[2] & 0x7F) << 7) | (rx[3] & 0x7F);
}

// パラメータ書き込み（SC 0x01-0x04。EEPROMにも保存される）。成功時true
bool icsWriteParam(u8 id, u8 sc, u8 value)
{
  u8 tx[3] = {(u8)(ICS_CMD_WRITE | id), sc, value};
  u8 rx[3];
  if (!icsTransfer(tx, 3, rx, 3, RESPONSE_TIMEOUT))
    return false;
  return rx[0] == (0x40 | id) && rx[1] == sc && rx[2] == value;
}

// EEPROM読み出し（64バイトのニブル列）。成功時true
bool icsReadEeprom(u8 id, u8 *eeprom)
{
  u8 tx[2] = {(u8)(ICS_CMD_READ | id), ICS_SC_EEPROM};
  u8 rx[2 + EEPROM_SIZE];
  if (!icsTransfer(tx, 2, rx, sizeof(rx), RESPONSE_TIMEOUT))
    return false;
  if (rx[0] != (0x20 | id) || rx[1] != ICS_SC_EEPROM)
    return false;
  memcpy(eeprom, &rx[2], EEPROM_SIZE);
  return true;
}

// EEPROM書き込み（64バイトを一括）。成功時true
bool icsWriteEeprom(u8 id, const u8 *eeprom)
{
  u8 tx[2 + EEPROM_SIZE];
  tx[0] = ICS_CMD_WRITE | id;
  tx[1] = ICS_SC_EEPROM;
  memcpy(&tx[2], eeprom, EEPROM_SIZE);
  u8 rx[2];
  if (!icsTransfer(tx, sizeof(tx), rx, 2, RESPONSE_TIMEOUT))
    return false;
  return rx[0] == (0x40 | id) && rx[1] == ICS_SC_EEPROM;
}

// ID読み出し（1台のみ接続）。失敗時 -1
int icsReadId()
{
  u8 tx[4] = {ICS_CMD_ID_READ, 0x00, 0x00, 0x00};
  u8 rx[1];
  bool ok = icsTransfer(tx, 4, rx, 1, ID_CMD_TIMEOUT);
  delay(ID_CMD_WAIT); // IDコマンドの後は500ms以上たたないと次のコマンドに応答しない
  if (!ok)
    return -1;
  return rx[0] & 0x1F; // IDコマンドの返信はMSBがマスクされない
}

// ID書き込み（1台のみ接続）。成功時は返信のID（失敗時 -1）
int icsWriteId(u8 id)
{
  u8 tx[4] = {(u8)(ICS_CMD_ID | id), 0x01, 0x01, 0x01};
  u8 rx[1];
  bool ok = icsTransfer(tx, 4, rx, 1, ID_CMD_TIMEOUT);
  delay(ID_CMD_WAIT); // IDコマンドの後は500ms以上たたないと次のコマンドに応答しない
  if (!ok)
    return -1;
  return rx[0] & 0x1F;
}

// ===== EEPROM ヘルパー =====

// ニブル列から1バイト値を取り出す
u8 eeGet8(const u8 *ee, u8 offset)
{
  return ((ee[offset] & 0x0F) << 4) | (ee[offset + 1] & 0x0F);
}

// 1バイト値をニブル列に格納
void eeSet8(u8 *ee, u8 offset, u8 value)
{
  ee[offset] = value >> 4;
  ee[offset + 1] = value & 0x0F;
}

// ニブル列から2バイト値を取り出す（上位バイト上位4bitから順）
u16 eeGet16(const u8 *ee, u8 offset)
{
  return ((u16)eeGet8(ee, offset) << 8) | eeGet8(ee, offset + 2);
}

// 2バイト値をニブル列に格納
void eeSet16(u8 *ee, u8 offset, u16 value)
{
  eeSet8(ee, offset, value >> 8);
  eeSet8(ee, offset + 2, value & 0xFF);
}

// 読み出したEEPROMが正しい形式か（先頭0x5A, 全バイトが4bit以内）
bool eepromLooksValid(const u8 *ee)
{
  if (eeGet8(ee, 0) != EEPROM_BACKUP_CHAR)
    return false;
  for (u8 i = 0; i < EEPROM_SIZE; i++)
  {
    if (ee[i] > 0x0F)
      return false;
  }
  return true;
}

// EEPROMの通信速度値を、ESP32のボーレートインデックスに変換（不明は -1）
int eepromBaudToIndex(u8 value)
{
  if (value == EE_BAUD_115K_DEC || value == EE_BAUD_115K_HEX)
    return 0;
  if (value == EE_BAUD_625K)
    return 1;
  if (value == EE_BAUD_1250K)
    return 2;
  return -1;
}

// EEPROMを安全に書き換え（読み出し → 形式確認 → 変更 → 一括書き込み → 読み返し照合）
// modify: 読み出したイメージを書き換える関数。falseを返すと中止
bool modifyEeprom(u8 id, bool (*modify)(u8 *ee, void *ctx), void *ctx)
{
  u8 ee[EEPROM_SIZE];
  if (!icsReadEeprom(id, ee))
  {
    Serial.println("Error: EEPROM read failed");
    return false;
  }
  if (!eepromLooksValid(ee))
  {
    Serial.println("Error: EEPROM data looks invalid (header 0x5A / nibble check). Aborted.");
    return false;
  }

  u8 newEe[EEPROM_SIZE];
  memcpy(newEe, ee, EEPROM_SIZE);
  if (!modify(newEe, ctx))
    return false;
  if (memcmp(ee, newEe, EEPROM_SIZE) == 0)
  {
    Serial.println("No change. Nothing written.");
    return true;
  }

  if (!icsWriteEeprom(id, newEe))
  {
    Serial.println("Error: EEPROM write failed (no reply). Read it again with [l] to check.");
    return false;
  }
  delay(50);

  u8 verify[EEPROM_SIZE];
  if (!icsReadEeprom(id, verify) || memcmp(verify, newEe, EEPROM_SIZE) != 0)
  {
    Serial.println("[Error] Verification failed. Read it again with [l] to check.");
    return false;
  }
  Serial.println("Verification: EEPROM [OK]");
  return true;
}

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

// 位置から角度への変換（7500 = 0度）
float positionToAngle(int pos)
{
  return (pos - ICS_POS_CENTER) / ICS_COUNT_PER_DEG;
}

// 角度から位置への変換（3500-11500に制限）
u16 angleToPosition(float angle)
{
  long pos = lroundf(ICS_POS_CENTER + angle * ICS_COUNT_PER_DEG);
  return constrain(pos, ICS_POS_MIN, ICS_POS_MAX);
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
  bool selected[ICS_ID_MAX + 1] = {false};
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
    if (first < ICS_ID_MIN || last > ICS_ID_MAX || first > last)
      return -1;

    for (int id = first; id <= last; id++)
      selected[id] = true;
  }

  int count = 0;
  for (int id = ICS_ID_MIN; id <= ICS_ID_MAX; id++)
  {
    if (selected[id])
      out[count++] = id;
  }
  return count > 0 ? count : -1;
}

// ===== コマンド =====

// 指定範囲のIDを検索（スピード読み出しで確認。サーボは動かない）。見つかった数を返す
// ics36: NULLでなければ、各サーボがICS3.6か（位置読み出しに応答したか）を格納（showDetail時のみ判定）
u8 scanServos(int maxId, u8 *found, bool showDetail, bool *ics36 = NULL)
{
  u8 count = 0;
  for (int id = ICS_ID_MIN; id <= maxId; id++)
  {
    int speed = icsReadParam(id, ICS_SC_SPEED, SCAN_TIMEOUT);
    if (speed < 0)
      continue;

    if (ics36 != NULL)
      ics36[count] = false;
    found[count++] = id;
    if (showDetail)
    {
      int stretch = icsReadParam(id, ICS_SC_STRETCH, SCAN_TIMEOUT);
      int pos = icsReadPosition(id, SCAN_TIMEOUT);
      if (ics36 != NULL)
        ics36[count - 1] = (pos >= 0);
      Serial.print("Found servo ID: ");
      Serial.print(id);
      Serial.print(" (");
      Serial.print(pos >= 0 ? "ICS3.6" : "ICS3.5");
      Serial.print(", speed ");
      Serial.print(speed);
      Serial.print(", stretch ");
      Serial.print(stretch);
      if (pos >= 0)
      {
        Serial.print(", pos ");
        Serial.print(pos);
      }
      Serial.println(")");
    }
  }
  return count;
}

// 検索結果をターゲットと一斉制御リストに設定
void applyScanResult(const u8 *found, u8 count)
{
  Serial.print("Found ");
  Serial.print(count);
  Serial.print(" servo(s) at ");
  Serial.print(ICS_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");

  memcpy(syncServoIDs, found, count);
  syncServoCount = count;
  targetServoID = found[0];
  Serial.print("Target ID auto-set to: ");
  Serial.println(targetServoID);
  Serial.print("Sync servo IDs auto-set to: ");
  printSyncServoIDs();
  Serial.println();
}

// 今の通信速度でID0-maxIdを検索し、見つかればターゲットと一斉制御リストに設定。見つかった数を返す
u8 scanAtCurrentBaud(int maxId, u8 *found, bool *ics36)
{
  Serial.print("Scanning ID0 to ID");
  Serial.print(maxId);
  Serial.print(" at ");
  Serial.print(ICS_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps...");

  u8 count = scanServos(maxId, found, true, ics36);
  if (count > 0)
    applyScanResult(found, count);
  return count;
}

// [f]コマンド: サーボ認識（見つからなければ全ボーレート検索を提案）
void executeServoDetectionCommand()
{
  u8 found[MAX_SYNC_SERVOS];

  Serial.println("Enter maximum ID to scan (0-31, empty = 31): ");
  String input = readInput();
  long maxId = ICS_ID_MAX;
  if (input.length() > 0 && (!parseNumber(input, &maxId) || maxId < ICS_ID_MIN || maxId > ICS_ID_MAX))
  {
    Serial.println("Error: ID out of range (0-31)");
    return;
  }

  if (scanAtCurrentBaud(maxId, found, NULL) > 0)
    return;

  Serial.println("No servos detected");
  Serial.println("Press Enter to scan all baud rates (fastest first), or type any key + Enter to cancel:");
  if (readInput().length() != 0)
  {
    Serial.println("Canceled");
    return;
  }

  // 速い順に全ボーレートを検索し、最も多く見つかった速度にする
  u8 original = servoBaudIndex;
  int bestIndex = -1;
  u8 bestCount = 0;
  u8 bestFound[MAX_SYNC_SERVOS];
  for (int b = ICS_BAUD_COUNT - 1; b >= 0; b--)
  {
    setHostBaudRate(b);
    u8 n = scanServos(maxId, found, false);
    Serial.print("  ");
    Serial.print(ICS_BAUD_RATES[b]);
    Serial.print(" bps -> ");
    Serial.print(n);
    Serial.println(" servo(s)");
    if (n > bestCount)
    {
      bestCount = n;
      bestIndex = b;
      memcpy(bestFound, found, n);
    }
  }

  if (bestIndex < 0)
  {
    setHostBaudRate(original);
    Serial.println("No servos detected at any baud rate. Check wiring/power (see README).");
    return;
  }
  setHostBaudRate(bestIndex);
  Serial.print("ESP32 baud rate auto-set to: ");
  Serial.print(ICS_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  applyScanResult(bestFound, bestCount);
}

// [z]コマンド: ターゲットID変更
void executeChangeTargetIdCommand()
{
  Serial.print("Current target ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter new target ID (0-31): ");

  long newId;
  if (!parseNumber(readInput(), &newId))
    newId = -1; // 範囲外としてエラー表示
  apiSetTarget(newId);
}

// [n]コマンド: 一斉制御サーボIDリスト設定
void executeSetSyncServoIDsCommand()
{
  Serial.print("Current sync servo IDs: ");
  printSyncServoIDs();
  Serial.println();
  Serial.println("Enter sync servo IDs (e.g. 0,3,7 or 0-4,7): ");

  u8 newIDs[MAX_SYNC_SERVOS];
  int newCount = parseIdList(readInput(), newIDs);
  if (newCount < 0)
    newCount = 0; // 不正としてエラー表示
  apiSetSync(newIDs, newCount);
}

// [r]コマンド: 現在位置読み取り（ICS3.6）
void executeReadPositionCommand()
{
  float deg;
  apiReadPosition(targetServoID, &deg);
}

// [v]コマンド: 電流・温度読み取り
void executeReadCurrentTempCommand()
{
  Serial.print("Reading current and temperature from servo ID ");
  Serial.println(targetServoID);

  int current = icsReadParam(targetServoID, ICS_SC_CURRENT);
  int temp = icsReadParam(targetServoID, ICS_SC_TEMP);
  if (current < 0 && temp < 0)
  {
    Serial.println("Error: No response (KRS-3301/3302 cannot read current/temperature)");
    return;
  }

  if (current >= 0)
  {
    Serial.print("Current    : ");
    Serial.print(current);
    Serial.print(" (approx. ");
    Serial.print((current & 0x3F) * 0.1, 1);
    Serial.print("A, ");
    Serial.print((current & 0x40) ? "reverse" : "forward");
    Serial.println(")");
  }
  if (temp >= 0)
  {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" (smaller is hotter. 60 = approx. 80°C, 30 = approx. 100°C)");
  }
}

// [a]コマンド: 全サーボ位置読み出し（ICS3.6）
void executeReadAllPositionsCommand()
{
  Serial.print("Reading positions (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");
  Serial.println("ID\tPosition\tAngle(°)");
  Serial.println("-------------------------");

  u8 ok = 0;
  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    int pos = icsReadPosition(id);
    Serial.print(id);
    if (pos >= 0)
    {
      Serial.print("\t");
      Serial.print(pos);
      Serial.print("\t\t");
      Serial.println(positionToAngle(pos), 1);
      ok++;
    }
    else
    {
      Serial.println("\t[Error] No response (ICS3.6 only)");
    }
  }
  Serial.println("-------------------------");
  Serial.print("Read: ");
  Serial.print(ok);
  Serial.print("/");
  Serial.println(syncServoCount);
}

// 一斉制御リストの全サーボに同じ位置を送る。応答のあった数を返す
u8 moveSyncServos(u16 pos)
{
  u8 ok = 0;
  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    int reply = icsSetPosition(id, pos);
    Serial.print("ID ");
    Serial.print(id);
    Serial.print(": target ");
    Serial.print(pos);
    Serial.print(" (");
    Serial.print(positionToAngle(pos), 1);
    Serial.print("°)");
    if (reply >= 0)
    {
      Serial.print(", current ");
      Serial.println(reply);
      ok++;
    }
    else
    {
      Serial.println(", No response");
    }
  }
  return ok;
}

// [c]コマンド: 全サーボ中央
void executeResetAllPositionsCommand()
{
  apiCenter();
}

// [x]コマンド: 全サーボに角度を送信
void executeAngleCommand()
{
  Serial.println("Enter angle (-135 to 135 degrees): ");

  String input = readInput();
  if (input.length() == 0)
  {
    Serial.println("Error: Angle out of range (-135 to 135)");
    return;
  }
  apiMove(input.toFloat());
}

// 一斉制御リストの全サーボを脱力（位置0。返信で現在位置を表示）。応答のあった数を返す
u8 freeSyncServos()
{
  u8 ok = 0;
  Serial.println("Free (torque off) all servos in the list...");
  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    int pos = icsSetPosition(id, ICS_POS_FREE);
    Serial.print("ID ");
    Serial.print(id);
    if (pos >= 0)
    {
      Serial.print(": free, position ");
      Serial.print(pos);
      Serial.print(" (");
      Serial.print(positionToAngle(pos), 1);
      Serial.println("°)");
      ok++;
    }
    else
    {
      Serial.println(": No response");
    }
  }
  return ok;
}

// [d]コマンド: 全サーボ脱力
void executeFreeCommand()
{
  apiTorque(false);
}

// 一斉制御リストの全サーボを現在位置で保持（脱力して現在位置を取得 → その位置を目標に）。保持できた数を返す
u8 holdSyncServos()
{
  u8 ok = 0;
  Serial.println("Hold all servos at the current position...");
  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    int pos = icsSetPosition(id, ICS_POS_FREE);
    Serial.print("ID ");
    Serial.print(id);
    if (pos < ICS_POS_MIN || pos > ICS_POS_MAX)
    {
      Serial.println(pos < 0 ? ": No response" : ": Position out of range, left free");
      continue;
    }
    icsSetPosition(id, pos);
    Serial.print(": hold at ");
    Serial.print(pos);
    Serial.print(" (");
    Serial.print(positionToAngle(pos), 1);
    Serial.println("°)");
    ok++;
  }
  return ok;
}

// [e]コマンド: 全サーボを現在位置で保持
void executeHoldCommand()
{
  apiTorque(true);
}

// [p]コマンド: パラメータ読み出し（ストレッチ・スピード・電流制限ではなく現在値の読み出し）
void executeReadParamsCommand()
{
  Serial.print("Reading parameters from servo ID ");
  Serial.println(targetServoID);

  const char *names[] = {"", "Stretch", "Speed", "Current", "Temperature"};
  for (u8 sc = ICS_SC_STRETCH; sc <= ICS_SC_TEMP; sc++)
  {
    int value = icsReadParam(targetServoID, sc);
    Serial.print(names[sc]);
    Serial.print(sc == ICS_SC_TEMP ? "\t: " : "\t\t: ");
    if (value < 0)
      Serial.println("No response");
    else
      Serial.println(value);
  }
}

// [s]コマンド: パラメータ書き込み（SC 0x01-0x04。EEPROMにも保存される）
void executeWriteParamCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  Serial.println("Select: [1] Stretch (1-127)  [2] Speed (1-127)  [3] Current limit (1-63)  [4] Temperature limit (1-127)");

  long sc;
  if (!parseNumber(readInput(), &sc) || sc < ICS_SC_STRETCH || sc > ICS_SC_TEMP)
  {
    Serial.println("Error: Invalid selection");
    return;
  }

  long maxValue = (sc == ICS_SC_CURRENT) ? 63 : 127;
  Serial.print("Enter value (1-");
  Serial.print(maxValue);
  Serial.println("): ");
  long value;
  if (!parseNumber(readInput(), &value) || value < 1 || value > maxValue)
  {
    Serial.println("Error: Value out of range");
    return;
  }

  if (icsWriteParam(targetServoID, sc, value))
    Serial.println("Written [OK] (also saved to EEPROM)");
  else
    Serial.println("Error: Write failed (no reply or reply mismatch)");
}

// EEPROMの内容を表示
void printEeprom(const u8 *ee)
{
  u8 flags = eeGet8(ee, EE_FLAGS);
  u8 baud = eeGet8(ee, EE_BAUD);
  int baudIndex = eepromBaudToIndex(baud);

  Serial.print("Stretch gain      : ");
  Serial.println(eeGet8(ee, EE_STRETCH));
  Serial.print("Speed             : ");
  Serial.println(eeGet8(ee, EE_SPEED));
  Serial.print("Punch             : ");
  Serial.println(eeGet8(ee, EE_PUNCH));
  Serial.print("Dead band         : ");
  Serial.println(eeGet8(ee, EE_DEAD_BAND));
  Serial.print("Damping           : ");
  Serial.println(eeGet8(ee, EE_DAMPING));
  Serial.print("Safe timer        : ");
  Serial.println(eeGet8(ee, EE_SAFE_TIMER));
  Serial.print("Flags             : 0x");
  Serial.print(flags, HEX);
  Serial.print(" (reverse ");
  Serial.print((flags & FLAG_REVERSE) ? "ON" : "OFF");
  Serial.print(", PWMINH ");
  Serial.print((flags & FLAG_PWMINH) ? "ON" : "OFF");
  Serial.print(", rotation ");
  Serial.print((flags & FLAG_ROTATION) ? "ON" : "OFF");
  Serial.print(", slave ");
  Serial.print((flags & FLAG_SLAVE) ? "ON" : "OFF");
  Serial.println(")");
  Serial.print("Limit upper/lower : ");
  Serial.print(eeGet16(ee, EE_LIMIT_UPPER));
  Serial.print(" / ");
  Serial.println(eeGet16(ee, EE_LIMIT_LOWER));
  Serial.print("Baud rate         : 0x");
  Serial.print(baud, HEX);
  Serial.print(" (");
  if (baudIndex >= 0)
    Serial.print(ICS_BAUD_RATES[baudIndex]);
  else
    Serial.print("unknown");
  Serial.println(" bps)");
  Serial.print("Temperature limit : ");
  Serial.println(eeGet8(ee, EE_TEMP_LIMIT));
  Serial.print("Current limit     : ");
  Serial.println(eeGet8(ee, EE_CURRENT_LIMIT));
  Serial.print("Response          : ");
  Serial.println(eeGet8(ee, EE_RESPONSE));
  Serial.print("User offset       : ");
  Serial.println((signed char)eeGet8(ee, EE_USER_OFFSET));
  Serial.print("ID                : ");
  Serial.println(eeGet8(ee, EE_ID));
  Serial.print("Char. stretch 1/2/3: ");
  Serial.print(eeGet8(ee, EE_CHAR_STRETCH1));
  Serial.print(" / ");
  Serial.print(eeGet8(ee, EE_CHAR_STRETCH2));
  Serial.print(" / ");
  Serial.println(eeGet8(ee, EE_CHAR_STRETCH3));
}

// [l]コマンド: EEPROM設定の読み出し
void executeReadEepromCommand()
{
  Serial.print("Reading EEPROM from servo ID ");
  Serial.println(targetServoID);

  u8 ee[EEPROM_SIZE];
  if (!icsReadEeprom(targetServoID, ee))
  {
    Serial.println("Error: No response");
    return;
  }
  if (!eepromLooksValid(ee))
    Serial.println("[Warning] EEPROM data looks invalid (header 0x5A / nibble check)");
  printEeprom(ee);
}

// EEPROM編集項目
struct EepromItem
{
  const char *name;
  u8 offset;
  u8 size;      // 1 or 2バイト
  long minValue;
  long maxValue;
  bool even;    // ストレッチ系は偶数のみ
  bool isSigned;
};
const EepromItem EEPROM_ITEMS[] = {
    {"Stretch gain (even)", EE_STRETCH, 1, 2, 254, true, false},
    {"Speed", EE_SPEED, 1, 1, 127, false, false},
    {"Punch", EE_PUNCH, 1, 0, 10, false, false},
    {"Dead band", EE_DEAD_BAND, 1, 0, 5, false, false},
    {"Damping", EE_DAMPING, 1, 1, 255, false, false},
    {"Safe timer", EE_SAFE_TIMER, 1, 10, 255, false, false},
    {"Limit upper", EE_LIMIT_UPPER, 2, ICS_POS_MIN, ICS_POS_MAX, false, false},
    {"Limit lower", EE_LIMIT_LOWER, 2, ICS_POS_MIN, ICS_POS_MAX, false, false},
    {"Temperature limit", EE_TEMP_LIMIT, 1, 1, 127, false, false},
    {"Current limit", EE_CURRENT_LIMIT, 1, 1, 63, false, false},
    {"Response", EE_RESPONSE, 1, 1, 5, false, false},
    {"User offset", EE_USER_OFFSET, 1, -127, 127, false, true},
    {"Char. stretch 1 (even)", EE_CHAR_STRETCH1, 1, 2, 254, true, false},
    {"Char. stretch 2 (even)", EE_CHAR_STRETCH2, 1, 2, 254, true, false},
    {"Char. stretch 3 (even)", EE_CHAR_STRETCH3, 1, 2, 254, true, false},
};
#define EEPROM_ITEM_COUNT (sizeof(EEPROM_ITEMS) / sizeof(EEPROM_ITEMS[0]))

// 編集内容（modifyEepromに渡す）
struct EepromEdit
{
  const EepromItem *item; // 数値項目（NULLならフラグ）
  long value;
  u8 flagMask;
  bool flagOn;
};

// 編集内容をEEPROMイメージに反映
bool applyEepromEdit(u8 *ee, void *ctx)
{
  EepromEdit *edit = (EepromEdit *)ctx;
  if (edit->item == NULL)
  {
    u8 flags = eeGet8(ee, EE_FLAGS) | FLAG_FIXED_ONE; // bit2は常に1
    flags = edit->flagOn ? (flags | edit->flagMask) : (flags & ~edit->flagMask);
    eeSet8(ee, EE_FLAGS, flags);
    return true;
  }

  if (edit->item->size == 2)
    eeSet16(ee, edit->item->offset, edit->value);
  else
    eeSet8(ee, edit->item->offset, (u8)edit->value); // 符号付きは2の補数

  // リミットは上限 >= 下限であること
  if (eeGet16(ee, EE_LIMIT_UPPER) < eeGet16(ee, EE_LIMIT_LOWER))
  {
    Serial.println("Error: Limit upper must be >= limit lower");
    return false;
  }
  return true;
}

// [k]コマンド: EEPROM設定の編集（読み出し → 1項目変更 → 一括書き込み → 照合）
void executeEditEepromCommand()
{
  Serial.print("Edit EEPROM of servo ID ");
  Serial.println(targetServoID);
  for (u8 i = 0; i < EEPROM_ITEM_COUNT; i++)
  {
    Serial.print("  [");
    Serial.print(i + 1);
    Serial.print("] ");
    Serial.print(EEPROM_ITEMS[i].name);
    Serial.print(" (");
    Serial.print(EEPROM_ITEMS[i].minValue);
    Serial.print("-");
    Serial.print(EEPROM_ITEMS[i].maxValue);
    Serial.println(")");
  }
  Serial.println("  [16] Reverse flag  [17] PWMINH (serial only) flag  [18] Rotation mode flag  [19] Slave flag");
  Serial.println("Select item: ");

  long index;
  if (!parseNumber(readInput(), &index) || index < 1 || index > (long)EEPROM_ITEM_COUNT + 4)
  {
    Serial.println("Error: Invalid selection");
    return;
  }

  EepromEdit edit = {NULL, 0, 0, false};
  if (index > (long)EEPROM_ITEM_COUNT)
  {
    const u8 masks[] = {FLAG_REVERSE, FLAG_PWMINH, FLAG_ROTATION, FLAG_SLAVE};
    edit.flagMask = masks[index - EEPROM_ITEM_COUNT - 1];
    Serial.println("Enter [1] ON or [0] OFF: ");
    String input = readInput();
    if (input != "0" && input != "1")
    {
      Serial.println("Error: Invalid selection");
      return;
    }
    edit.flagOn = (input == "1");
    if (edit.flagMask == FLAG_SLAVE && edit.flagOn)
      Serial.println("[Warning] Slave servos do not reply. This tool cannot read them afterwards.");
  }
  else
  {
    edit.item = &EEPROM_ITEMS[index - 1];
    Serial.print("Enter value (");
    Serial.print(edit.item->minValue);
    Serial.print("-");
    Serial.print(edit.item->maxValue);
    Serial.println("): ");
    if (!parseNumber(readInput(), &edit.value) || edit.value < edit.item->minValue ||
        edit.value > edit.item->maxValue || (edit.item->even && (edit.value % 2) != 0))
    {
      Serial.println("Error: Value out of range (stretch values must be even)");
      return;
    }
  }

  if (modifyEeprom(targetServoID, applyEepromEdit, &edit))
    Serial.println("Done. Power-cycle the servo if the change does not take effect.");
}

// [o]コマンド: 回転方向（リバースフラグ）の設定
void executeSetDirectionCommand()
{
  Serial.println("Select direction: [0] Normal  [1] Reversed");
  String input = readInput();
  if (input != "0" && input != "1")
  {
    Serial.println("Error: Invalid selection");
    return;
  }
  EepromEdit edit = {NULL, 0, FLAG_REVERSE, input == "1"};
  if (modifyEeprom(targetServoID, applyEepromEdit, &edit))
    Serial.println("Direction saved.");
}

// 通信速度の書き換え内容（modifyEepromに渡す）
bool applyBaudEdit(u8 *ee, void *ctx)
{
  u8 baudIndex = *(u8 *)ctx;
  u8 current = eeGet8(ee, EE_BAUD);
  u8 value;
  if (baudIndex == 1)
    value = EE_BAUD_625K;
  else if (baudIndex == 2)
    value = EE_BAUD_1250K;
  else
    value = (current == EE_BAUD_115K_HEX) ? EE_BAUD_115K_HEX : EE_BAUD_115K_DEC; // 実機の表記に合わせる

  if (eepromBaudToIndex(current) < 0)
  {
    Serial.print("Error: Unknown baud value 0x");
    Serial.print(current, HEX);
    Serial.println(" in EEPROM. Aborted.");
    return false;
  }
  eeSet8(ee, EE_BAUD, value);
  return true;
}

// [b]コマンド: 通信速度設定
void executeSetBaudRateCommand()
{
  Serial.print("Current ESP32 baud rate: ");
  Serial.print(ICS_BAUD_RATES[servoBaudIndex]);
  Serial.println(" bps");
  for (u8 i = 0; i < ICS_BAUD_COUNT; i++)
  {
    Serial.print("  [");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(ICS_BAUD_RATES[i]);
    Serial.println(" bps");
  }
  Serial.println("Enter baud rate number (0-2): ");
  String baudInput = readInput();
  if (baudInput != "0" && baudInput != "1" && baudInput != "2")
  {
    Serial.println("Error: Invalid selection");
    return;
  }
  u8 baudIndex = baudInput.toInt();

  Serial.println("Apply to: [1] ESP32 only");
  Serial.print("          [2] Servo ID ");
  Serial.print(targetServoID);
  Serial.println(" (EEPROM) + ESP32");
  String mode = readInput();

  if (mode != "1" && mode != "2")
  {
    Serial.println("Error: Invalid selection");
    return;
  }
  apiSetBaud(baudIndex, mode == "2");
}

// サーボID変更の前半: 1台だけの接続を促し、今のIDを読み出す（失敗時 -1）
int readCurrentServoId()
{
  Serial.println("[Warning] Connect only ONE servo (ID commands are received by all servos).");
  Serial.println("Reading current ID...");

  int currentId = icsReadId();
  if (currentId < 0)
  {
    Serial.println("Error: No response to ID read");
    return -1;
  }
  Serial.print("Current servo ID: ");
  Serial.println(currentId);
  return currentId;
}

// サーボID変更の後半: 新しいIDを書き込んで読み返し、ターゲットと一斉制御リストに設定。成功時true
bool writeServoId(int currentId, long newId)
{
  if (newId < ICS_ID_MIN || newId > ICS_ID_MAX)
  {
    Serial.println("Error: ID out of range (0-31)");
    return false;
  }
  if (newId == currentId)
  {
    Serial.println("New ID is the same as current ID. Nothing to do.");
    return true;
  }

  Serial.print("Changing servo ID ");
  Serial.print(currentId);
  Serial.print(" -> ");
  Serial.println(newId);

  int reply = icsWriteId(newId);
  if (reply != newId)
  {
    Serial.println("Error: ID write failed (no reply or reply mismatch)");
    return false;
  }
  // 読み返して確認（応答が遅れることがあるため3回まで）
  int readBack = -1;
  for (int retry = 0; retry < 3 && readBack != newId; retry++)
    readBack = icsReadId();
  if (readBack == newId)
  {
    Serial.println("Verification: ID [OK]");
    targetServoID = newId;
    syncServoIDs[0] = newId;
    syncServoCount = 1;
    Serial.print("Target ID and sync servo IDs set to: ");
    Serial.println(newId);
    return true;
  }
  Serial.print("[Error] Verification failed (read ID: ");
  Serial.print(readBack);
  Serial.println("). Power-cycle the servo and use [i] again to check.");
  return false;
}

// [i]コマンド: サーボID変更（1台のみ接続）
void executeChangeServoIdCommand()
{
  int currentId = readCurrentServoId();
  if (currentId < 0)
    return;
  Serial.println("Enter new servo ID (0-31): ");

  long newId;
  if (!parseNumber(readInput(), &newId))
    newId = -1; // 範囲外としてエラー表示
  writeServoId(currentId, newId);
}

// [t]コマンド: 送信テスト（ID0-31全員に位置を送り往復。動けば送信OK、返信のあったIDも表示）
void executeTxTestCommand()
{
  Serial.println("TX test: sending position to ALL IDs (0-31): +30, -30, +30, -30, center.");
  Serial.println("If the servo moves, TX (ESP32 -> servo) is OK.");

  const float angles[] = {30.0, -30.0, 30.0, -30.0, 0.0};
  bool replied[ICS_ID_MAX + 1] = {false};
  for (u8 step = 0; step < sizeof(angles) / sizeof(angles[0]); step++)
  {
    u16 pos = angleToPosition(angles[step]);
    for (int id = ICS_ID_MIN; id <= ICS_ID_MAX; id++)
    {
      if (icsSetPosition(id, pos, SCAN_TIMEOUT) >= 0)
        replied[id] = true;
    }
    Serial.print("  ");
    Serial.print(angles[step], 0);
    Serial.println("deg sent");
    delay(700);
  }

  Serial.print("Replied IDs: ");
  u8 count = 0;
  for (int id = ICS_ID_MIN; id <= ICS_ID_MAX; id++)
  {
    if (!replied[id])
      continue;
    if (count > 0)
      Serial.print(",");
    Serial.print(id);
    count++;
  }
  Serial.println(count == 0 ? "none" : "");

  Serial.println("TX test finished.");
  Serial.println("Moved, replied      -> TX and RX OK.");
  Serial.println("Moved, no reply     -> TX OK. Check RX path (use [g]).");
  Serial.println("Not moved           -> Check power, wiring, baud rate ([b] / [f]). Power-cycle the servo.");
}

// [g]コマンド: 受信テスト（ターゲットにスピード読み出しを送り、受信した生データを表示）
void executeRxDumpCommand()
{
  u8 tx[2] = {(u8)(ICS_CMD_READ | targetServoID), ICS_SC_SPEED};
  u8 raw[16];
  u8 count = 0;

  busFlushRx();
  busDirTx();
  servoSerial->write(tx, 2);
  busEndSendToRx();
  unsigned long start = millis();
  while (millis() - start < 20 && count < sizeof(raw))
  {
    if (servoSerial->available())
      raw[count++] = servoSerial->read();
  }

  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());
  Serial.print("Sent: ");
  Serial.print(tx[0], HEX);
  Serial.print(" ");
  Serial.println(tx[1], HEX);
  Serial.print("Received ");
  Serial.print(count);
  Serial.print(" byte(s): ");
  for (u8 i = 0; i < count; i++)
  {
    if (raw[i] < 0x10)
      Serial.print("0");
    Serial.print(raw[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Expected reply: ");
  if (busEchoEnabled)
    Serial.print("(echo A? 02) ");
  Serial.print(0x20 | targetServoID, HEX);
  Serial.println(" 02 <speed>");
}

void disp_menu()
{
  Serial.println("============================================");
  Serial.print("Current target ID: ");
  Serial.print(targetServoID);
  Serial.print(", Sync IDs: ");
  printSyncServoIDs();
  Serial.print(", Baud rate: ");
  Serial.print(ICS_BAUD_RATES[servoBaudIndex]);
  Serial.print(" bps 8E1, Echo: ");
  Serial.print(busEchoEnabled ? "ON" : "OFF");
  Serial.println(", Protocol: KONDO ICS3.5/3.6");
  Serial.println("Menu: [f] Find servos   [n] Set sync servo IDs   [z] Set target ID ");
  Serial.println("      [v] Read cur/temp [r] Read position        [p] Read params");
  Serial.println("      [c] All to center [a] Read all positions   [s] Write param");
  Serial.println("      [e] Hold position [d] Free (torque off)    [x] Send angle to all ");
  Serial.println("      [b] Set baud rate [i] Change servo ID      [l] Read EEPROM");
  Serial.println("      [k] Edit EEPROM   [o] Set direction        [g] RX dump (debug)");
  Serial.println("      [t] TX test (all IDs)");
  Serial.println("      [h] Change servo type (restart)");
  Serial.println("Input Command:");
}

// 初期化
void setup()
{
  Serial.begin(115200);
  beginServoBus();

  delay(1000);

  Serial.println("=====================================");
  Serial.println("Kondo KRS Servo Setter");
  Serial.println("KONDO ICS3.5/3.6 (KRS series)");
  Serial.println("=====================================");
  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());
  Serial.println("Use [f] to find servos first.");

  disp_menu();
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n'); // 改行を受けたらすぐ処理（ブラウザからの操作を速くする）
    command.trim();

    if (command == "f")
      executeServoDetectionCommand();
    else if (command == "z")
      executeChangeTargetIdCommand();
    else if (command == "n")
      executeSetSyncServoIDsCommand();
    else if (command == "r")
      executeReadPositionCommand();
    else if (command == "v")
      executeReadCurrentTempCommand();
    else if (command == "a")
      executeReadAllPositionsCommand();
    else if (command == "c")
      executeResetAllPositionsCommand();
    else if (command == "x")
      executeAngleCommand();
    else if (command == "d")
      executeFreeCommand();
    else if (command == "e")
      executeHoldCommand();
    else if (command == "p")
      executeReadParamsCommand();
    else if (command == "s")
      executeWriteParamCommand();
    else if (command == "l")
      executeReadEepromCommand();
    else if (command == "k")
      executeEditEepromCommand();
    else if (command == "o")
      executeSetDirectionCommand();
    else if (command == "b")
      executeSetBaudRateCommand();
    else if (command == "i")
      executeChangeServoIdCommand();
    else if (command == "g")
      executeRxDumpCommand();
    else if (command == "t")
      executeTxTestCommand();
    else if (command.startsWith("@"))
      handleApiCommand(command); // ブラウザ用の機械向けコマンド
    else if (command == "h")
      selectServoTypeAndRestart();
    else if (command.length() == 0)
      disp_menu();
    else
    {
      Serial.println("Unknown command. Available commands:");
      disp_menu();
    }
  }

  delay(50); // CPU負荷軽減
}

// ===== サーボ種類の自動判別（起動時のメニュー [0]） =====

// 読み取りコマンドだけでICS のサーボを探す（サーボは動かない）。
// bauds: 調べる通信速度（速い順に調べる）。見つかった通信速度（なければ0）
unsigned long detect(DetectBauds bauds)
{
  const u8 original = servoBaudIndex;
  unsigned long found = 0;
  busBegin(servoSerial, RX1, TX1, PIN_EN, ICS_BAUD_RATES[DEFAULT_BAUD_INDEX], ICS_SERIAL_CONFIG, false, false);
  for (int b = ICS_BAUD_COUNT - 1; b >= 0 && found == 0; b--)
  {
    if (detectSkipsBaud(bauds, b == DEFAULT_BAUD_INDEX, ICS_BAUD_RATES[b]))
      continue;
    setHostBaudRate(b);
    for (int id = ICS_ID_MIN; id <= ICS_ID_MAX; id++)
    {
      if (icsReadParam(id, ICS_SC_SPEED, SCAN_TIMEOUT) >= 0)
      {
        found = ICS_BAUD_RATES[b];
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
  for (u8 i = 0; i < ICS_BAUD_COUNT; i++)
  {
    if (ICS_BAUD_RATES[i] == baud)
      servoBaudIndex = i;
  }
}

// ===== ブラウザ用API =====
// 対話入力はせず、上の各コマンドと同じ処理をする（[x] などの対話コマンドもここを呼ぶ）

// 今の状態と、できること
void apiGetInfo(ApiInfo *info)
{
  info->idMin = ICS_ID_MIN;
  info->idMax = ICS_ID_MAX;
  info->angleMin = -ICS_ANGLE_LIMIT;
  info->angleMax = ICS_ANGLE_LIMIT;
  info->bauds = ICS_BAUD_RATES;
  info->baudCount = ICS_BAUD_COUNT;
  info->baudIndex = servoBaudIndex;
  info->baud = ICS_BAUD_RATES[servoBaudIndex];
  info->target = targetServoID;
  info->syncIds = syncServoIDs;
  info->syncCount = syncServoCount;
  info->oneServoForId = true; // IDコマンドはIDを含まないため1台だけ接続
}

// [f]の検索（今の通信速度だけ。全ボーレート検索はしない）。見つかればターゲットと一斉制御リストも設定
int apiScan(int maxId, ApiServo *found, int maxCount)
{
  maxId = constrain(maxId, ICS_ID_MIN, ICS_ID_MAX);
  u8 ids[MAX_SYNC_SERVOS];
  bool ics36[MAX_SYNC_SERVOS];
  u8 count = scanAtCurrentBaud(maxId, ids, ics36);
  if (count == 0)
    Serial.println("No servos detected");

  int n = constrain((int)count, 0, maxCount);
  for (int i = 0; i < n; i++)
  {
    found[i].id = ids[i];
    strlcpy(found[i].model, ics36[i] ? "ICS3.6" : "ICS3.5", API_MODEL_LEN);
  }
  return n;
}

// [z]: ターゲットID変更
bool apiSetTarget(int id)
{
  if (id < ICS_ID_MIN || id > ICS_ID_MAX)
  {
    Serial.println("Error: ID out of range (0-31)");
    return false;
  }
  targetServoID = id;
  Serial.print("Target ID changed to: ");
  Serial.println(targetServoID);
  return true;
}

// [n]: 一斉制御サーボIDリスト設定（昇順・重複なしにそろえる）
bool apiSetSync(const uint8_t *ids, uint8_t count)
{
  bool selected[ICS_ID_MAX + 1] = {false};
  for (u8 i = 0; i < count; i++)
  {
    if (ids[i] > ICS_ID_MAX)
      count = 0; // 範囲外のIDがあれば不正
    else
      selected[ids[i]] = true;
  }
  if (count == 0)
  {
    Serial.println("Error: Invalid ID list (valid IDs: 0-31)");
    return false;
  }

  syncServoCount = 0;
  for (int id = ICS_ID_MIN; id <= ICS_ID_MAX; id++)
  {
    if (selected[id])
      syncServoIDs[syncServoCount++] = id;
  }
  Serial.print("Sync servo IDs changed to: ");
  printSyncServoIDs();
  Serial.print(" (");
  Serial.print(syncServoCount);
  Serial.println(" servos)");
  return true;
}

// [x]: 一斉制御リストを角度へ（位置指令でトルクONになる）。1台でも応答すればtrue
bool apiMove(float deg)
{
  if (!(deg >= -ICS_ANGLE_LIMIT && deg <= ICS_ANGLE_LIMIT))
  {
    Serial.println("Error: Angle out of range (-135 to 135)");
    return false;
  }
  return moveSyncServos(angleToPosition(deg)) > 0;
}

// [c]: 一斉制御リストを中央へ。1台でも応答すればtrue
bool apiCenter()
{
  Serial.println("Moving all servos to center (7500)...");
  return moveSyncServos(ICS_POS_CENTER) > 0;
}

// [e] / [d]: ON = 現在位置で保持, OFF = 脱力。1台でもできればtrue
bool apiTorque(bool on)
{
  return (on ? holdSyncServos() : freeSyncServos()) > 0;
}

// [t]: 送信テスト
bool apiTxTest()
{
  executeTxTestCommand();
  return true;
}

// [r]: 現在位置読み取り（ICS3.6のみ。ICS3.5は失敗）
bool apiReadPosition(int id, float *deg)
{
  if (id < ICS_ID_MIN || id > ICS_ID_MAX)
  {
    Serial.println("Error: ID out of range (0-31)");
    return false;
  }
  Serial.print("Reading position from servo ID ");
  Serial.println(id);

  int pos = icsReadPosition(id);
  if (pos < 0)
  {
    Serial.println("Error: No response (current position read needs ICS3.6. For ICS3.5 use [d] to read while free)");
    return false;
  }
  *deg = positionToAngle(pos);
  Serial.print("Position: ");
  Serial.print(pos);
  Serial.print(" (");
  Serial.print(*deg, 1);
  Serial.println("°)");
  return true;
}

// [i]: サーボID変更（1台のみ接続。IDコマンドにIDがないため oldId は使わない）
bool apiSetId(int oldId, int newId)
{
  (void)oldId;
  int currentId = readCurrentServoId();
  if (currentId < 0)
    return false;
  return writeServoId(currentId, newId);
}

// [b]: 通信速度設定。servoToo: false = ESP32だけ, true = ターゲットのEEPROM + ESP32
bool apiSetBaud(int index, bool servoToo)
{
  if (index < 0 || index >= (int)ICS_BAUD_COUNT)
  {
    Serial.println("Error: Invalid selection");
    return false;
  }
  u8 baudIndex = index;

  if (!servoToo)
  {
    setHostBaudRate(baudIndex);
    Serial.print("ESP32 baud rate changed to: ");
    Serial.print(ICS_BAUD_RATES[servoBaudIndex]);
    Serial.println(" bps");
    return true;
  }

  if (!modifyEeprom(targetServoID, applyBaudEdit, &baudIndex))
    return false;

  // 新しい速度での応答を確認（反映タイミングはマニュアルに記載がないため両方を試す）
  setHostBaudRate(baudIndex);
  delay(100);
  if (icsReadParam(targetServoID, ICS_SC_SPEED) >= 0)
  {
    Serial.print("Servo responds at ");
    Serial.print(ICS_BAUD_RATES[servoBaudIndex]);
    Serial.println(" bps [OK]");
  }
  else
  {
    Serial.println("No response at the new baud rate yet. Power-cycle the servo, then use [f].");
  }
  return true; // EEPROMは書き込み・照合済み（反映に電源の入れ直しが必要な場合あり）
}

} // namespace kondo
