#include <Arduino.h>
#include "servo_bus.h"
#include <xtensa/hal.h>
#include "servo_select.h"

namespace hitec
{

// 基本型定義
typedef unsigned char u8;
typedef unsigned short u16;

// ピン定義
#define PIN_EN 33 // 半二重通信の送受信切替（HIGH: 送信, LOW: 受信）

// HMI通信定義（Hitec Multi-protocol Interface, シリアルモード）
#define HMI_BAUD_RATE 19200 // 通信速度（固定, 8N2）
#define HMI_HEADER 0x80     // パケット先頭
#define HMI_PACKET_SIZE 7   // Header, Command, Data1, Data2, Checksum, Received1, Received2
#define HMI_SEND_SIZE 5     // ESP32が駆動するバイト数（残り2バイトはサーボが返信を書き込む）

// HMIコマンド定義（"Hitec Robot Servos - HSR Serial Communication" 2007-09-23 より）
#define HMI_CMD_READ_EEPROM 0xE1  // EEPROM読み出し（Param1 = アドレス, Return1 = データ）
#define HMI_CMD_WRITE_EEPROM 0xE2 // EEPROM書き込み（Param1 = アドレス, Param2 = データ, 成功時 03 03）
#define HMI_CMD_VERSION_ID 0xE7   // バージョンとIDの読み出し（宛先IDなし）
#define HMI_CMD_PULSE_VOLT 0xE8   // モータ駆動パルス幅と電圧の読み出し（宛先IDなし）
#define HMI_CMD_SPEED_POS 0xE9    // 速度設定と位置読み出し（Param1 = ID, Param2 = 速度 1-255）
#define HMI_CMD_WRITE_MEMORY 0xE4 // RAM書き込み（Param1 = アドレス, Param2 = データ, 宛先IDなし）
#define HMI_CMD_PARAM_SET 0xEA    // 制御パラメータセット選択（Param2 = 1/2/3, 返信 03 06）
#define HMI_CMD_GO_STOP 0xEB      // Go/Stop（Param1 = 1/0。0で目標位置の変更を保留, 返信 03 06）
#define HMI_CMD_RELEASE 0xEF      // 脱力（位置制御停止, 返信 03 06。次の位置指令で全サーボ復帰）
// 位置設定はCommandバイト自体がサーボID（0x00-0x7F）

// EEPROM定義
#define EEPROM_ADDR_ID 0x29       // サーボID
#define EEPROM_ADDR_CHECKSUM 0x2C // EEPROMチェックサム（0x00-0x2Cの合計が256の倍数。不正だとモータが動かない）
#define EEPROM_WRITE_OK 0x03      // 書き込み成功時の返信（03 03）
#define MEMORY_ADDR_SETPOINT_LOW 0x06  // 現在の軌道目標値（下位, RAM）
#define MEMORY_ADDR_SETPOINT_HIGH 0x07 // 現在の軌道目標値（上位, RAM）
#define HMI_VOLTAGE_SCALE 0.03522 // 0xE8の電圧1あたりのV

// ID定義
#define HMI_ID_MIN 0
#define HMI_ID_MAX 127

// 位置定義（単位はパルス幅us, 1500 = 中央, 10us = 1度）
#define HMI_PULSE_CENTER 1500
#define HMI_PULSE_MIN 550  // 約-95度
#define HMI_PULSE_MAX 2450 // 約+95度
#define HMI_US_PER_DEGREE 10.0

// 速度定義（0xE9のParam2。有効範囲1-255。EEPROMの既定値は0xFF）
#define HMI_DEFAULT_SPEED 0xFF

// 一斉制御定義
#define MAX_SYNC_SERVOS 128

// 通信タイムアウト定義
#define REPLY_TIMEOUT 10 // 返信2バイトの受信待ち(ms)

// 通信結果
enum HmiResult
{
  HMI_OK,
  HMI_NO_RESPONSE,
  HMI_INVALID_RESPONSE
};

// グローバル変数
HardwareSerial *servoSerial = &Serial1;                 // 使用するシリアルポート
u8 targetServoID = 0;                                   // ターゲットサーボID
u8 syncServoIDs[MAX_SYNC_SERVOS] = {0};                 // 一斉制御するサーボIDリスト（とびとびIDも可）
u8 syncServoCount = 1;                                  // リスト内のサーボ数
u8 servoSpeed = HMI_DEFAULT_SPEED;                      // 0xE9で送る速度
bool signalInverted = true;                             // 線上の極性（HMI: アイドルLOW, スタートビットHIGH）
bool replyFramed = true;                                // 読み取りの返信の受け方（true: 区切り送信, false: 線を開放するだけ）

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

// 通信結果の表示用テキスト
const char *hmiResultText(HmiResult result)
{
  switch (result)
  {
  case HMI_OK:
    return "OK";
  case HMI_NO_RESPONSE:
    return "No response";
  default:
    return "Invalid response";
  }
}

// パルス幅から角度への変換（1500us = 0度, 10us = 1度）
float pulseToAngle(u16 pulseUs)
{
  return ((float)pulseUs - HMI_PULSE_CENTER) / HMI_US_PER_DEGREE;
}

// 角度からパルス幅への変換（550-2450usに制限）
u16 angleToPulse(float angle)
{
  long pulseUs = lroundf(HMI_PULSE_CENTER + angle * HMI_US_PER_DEGREE);
  return constrain(pulseUs, HMI_PULSE_MIN, HMI_PULSE_MAX);
}

// ===== HMI 通信レイヤー =====

// チェックサム計算（256 - (Byte1 + Byte2 + Byte3 + Byte4) の下位8ビット）
u8 hmiChecksum(const u8 *packet)
{
  u8 sum = packet[0] + packet[1] + packet[2] + packet[3];
  return (u8)(0x100 - sum);
}

// 送信部分（5バイト）の構築
void buildHmiPacket(u8 *packet, u8 command, u8 data1, u8 data2)
{
  packet[0] = HMI_HEADER;
  packet[1] = command;
  packet[2] = data1;
  packet[3] = data2;
  packet[4] = hmiChecksum(packet);
}

// シリアルポートの初期化（8N2, 極性反転の有無を反映, エコー自動判定, 送信モードで待機）
void beginServoSerial(bool restart)
{
  if (restart)
    servoSerial->end();
  busBegin(servoSerial, RX1, TX1, PIN_EN, HMI_BAUD_RATE, SERIAL_8N2, signalInverted);

  // 3ステートバッファ版では送信方向のときRXが浮くため、反転時のアイドル（LOW = ストップビット）に引いておく
  if (!busEchoEnabled && signalInverted)
    gpio_pulldown_en((gpio_num_t)RX1);
  else
    gpio_pulldown_dis((gpio_num_t)RX1);

  // 区切り送信でスタートビットを検出するため、TX端子の入力を有効にする
  gpio_ll_input_enable(&GPIO, (gpio_num_t)TX1);
}

// 返信2バイトを区切り送信で受ける（ホストが0x00を送り、サーボが必要なビットだけLOWに引く本来のHMIの動きを再現）
// 5バイトの送信完了後（送信方向のまま）に呼ぶ。6・7バイト目の0x00をUARTに直接書き込み、
// TX端子でスタートビット（HIGH）を検出した瞬間に線を開放し、最後のデータビットの読み取り後に送信方向へ戻す。
// ストップビット（LOW）はUART自身が出すため、区切りはUARTの実際の出力と一致する。終了時は送信方向
static void __attribute__((noinline)) IRAM_ATTR hmiFrameReplyBytes(uart_dev_t *hw, int8_t pinDir, int8_t pinTx,
                                                                   int8_t pinTxRelease, portMUX_TYPE *mux)
{
  const uint8_t zeros[2] = {0x00, 0x00};
  const uint32_t releaseUs = 455;          // スタート+データ8ビット(469us)のうち、最後のデータビットの読み取り後まで
  const uint32_t timeoutCycles = 240 * 3000; // 各待ちのタイムアウト（約3ms @240MHz）

  portENTER_CRITICAL(mux);
  uart_ll_write_txfifo(hw, zeros, 2);
  for (int i = 0; i < 2; i++)
  {
    // スタートビット（反転時HIGH）を待つ
    uint32_t start = xthal_get_ccount();
    while (!gpio_ll_get_level(&GPIO, (gpio_num_t)pinTx))
    {
      if (xthal_get_ccount() - start > timeoutCycles)
        break;
    }

    // 線を開放してサーボに書き込ませる
    busTxOutput(pinTxRelease, false);
    gpio_ll_set_level(&GPIO, (gpio_num_t)pinDir, 0);
    ets_delay_us(releaseUs);

    // 送信方向に戻す（この時点のTXはデータ0=HIGH、続いてUARTがストップビット=LOWを出す）
    gpio_ll_set_level(&GPIO, (gpio_num_t)pinDir, 1);
    busTxOutput(pinTxRelease, true);

    // ストップビット（LOW）に入るのを待つ（次のスタートビット検出のため）
    start = xthal_get_ccount();
    while (gpio_ll_get_level(&GPIO, (gpio_num_t)pinTx))
    {
      if (xthal_get_ccount() - start > timeoutCycles)
        break;
    }
  }
  portEXIT_CRITICAL(mux);
}

// 受信バッファフラッシュ
void flushReceiveBuffer()
{
  while (servoSerial->read() != -1)
    ;
}

// 受信した列から送信エコーを除き、返信2バイトを取り出す
HmiResult extractReply(const u8 *sent, const u8 *received, u8 count, u8 *reply1, u8 *reply2)
{
  u8 start = 0;
  if (count >= HMI_SEND_SIZE && memcmp(received, sent, HMI_SEND_SIZE) == 0)
    start = HMI_SEND_SIZE; // 回路によっては自分の送信が見える

  if (count - start < 2)
    return (count - start == 0) ? HMI_NO_RESPONSE : HMI_INVALID_RESPONSE;

  *reply1 = received[start];
  *reply2 = received[start + 1];
  return HMI_OK;
}

// HMIトランザクション（5バイト送信 → 線を開放 → サーボが書き込む2バイトを受信）
// raw/rawCount: 受信した生データ（診断用, NULL可）
HmiResult hmiTransaction(u8 command, u8 data1, u8 data2, u8 *reply1, u8 *reply2,
                         u8 *raw = NULL, u8 *rawCount = NULL)
{
  u8 packet[HMI_SEND_SIZE];
  buildHmiPacket(packet, command, data1, data2);

  // 受信バッファクリア → 送信モードで5バイト送信
  // → 6・7バイト目はサーボがLOWに引いて書き込むため、送信完了後すぐ線を開放（プルアップで待機）
  flushReceiveBuffer();
  busDirTx();
  servoSerial->write(packet, HMI_SEND_SIZE);
  if (replyFramed && signalInverted)
  {
    // 区切り送信: 送信方向のまま5バイトの完了を待ち、6・7バイト目をスタートビットに合わせて区切る
    while (!uart_ll_is_tx_idle(busUartHw))
    {
    }
    hmiFrameReplyBytes(busUartHw, busPinDir, TX1, busTxReleasePin, &busMux);
  }
  else
  {
    busEndSendToRx();
  }

  u8 received[16];
  u8 count = 0;
  unsigned long startTime = millis();
  while ((millis() - startTime) < REPLY_TIMEOUT && count < sizeof(received))
  {
    if (servoSerial->available())
      received[count++] = servoSerial->read();
  }

  // 送信モードに復帰（アイドルを駆動）
  busDirTx();

  if (raw && rawCount)
  {
    memcpy(raw, received, count);
    *rawCount = count;
  }

  u8 r1 = 0, r2 = 0;
  HmiResult result = extractReply(packet, received, count, &r1, &r2);
  if (reply1)
    *reply1 = r1;
  if (reply2)
    *reply2 = r2;
  return result;
}

// 位置設定（CommandバイトがサーボID。返信は0x00, 0x00）
// 実機で動作を確認できている「開放のみ」で送る（区切り送信の設定に関係なく）
void hmiSetPosition(u8 id, u16 pulseUs)
{
  bool framed = replyFramed;
  replyFramed = false;
  hmiTransaction(id & 0x7F, pulseUs >> 8, pulseUs & 0xFF, NULL, NULL);
  replyFramed = framed;
}

// バージョンとIDの読み出し（宛先IDなし: バス上は1台だけにすること）
HmiResult hmiReadVersionId(u8 *version, u8 *id)
{
  return hmiTransaction(HMI_CMD_VERSION_ID, 0x00, 0x00, version, id);
}

// 速度設定と位置読み出し（ID指定あり）
HmiResult hmiSetSpeedReadPosition(u8 id, u8 speed, u16 *pulseUs)
{
  u8 high, low;
  HmiResult result = hmiTransaction(HMI_CMD_SPEED_POS, id & 0x7F, speed, &high, &low);
  if (result == HMI_OK && pulseUs)
    *pulseUs = ((u16)high << 8) | low;
  return result;
}

// EEPROM 1バイト読み出し（0xE1）
HmiResult hmiReadEeprom(u8 address, u8 *data)
{
  u8 reply2;
  return hmiTransaction(HMI_CMD_READ_EEPROM, address, 0x00, data, &reply2);
}

// EEPROM 1バイト書き込み（0xE2）。成功時は 03 03 が返る
bool hmiWriteEeprom(u8 address, u8 data)
{
  u8 reply1, reply2;
  HmiResult result = hmiTransaction(HMI_CMD_WRITE_EEPROM, address, data, &reply1, &reply2);
  delay(10); // EEPROM書き込み時間
  return result == HMI_OK && reply1 == EEPROM_WRITE_OK && reply2 == EEPROM_WRITE_OK;
}

// EEPROM 0x00-0x2C を読み出し。成功時true
bool readEepromImage(u8 *image)
{
  for (u8 address = 0; address <= EEPROM_ADDR_CHECKSUM; address++)
  {
    if (hmiReadEeprom(address, &image[address]) != HMI_OK)
    {
      Serial.print("Error: EEPROM read failed at 0x");
      Serial.println(address, HEX);
      return false;
    }
  }
  return true;
}

// EEPROMチェックサム（0x2C）の正しい値: 256 - (0x00-0x2Bの合計) % 256
u8 eepromChecksum(const u8 *image)
{
  u8 sum = 0;
  for (u8 address = 0; address < EEPROM_ADDR_CHECKSUM; address++)
    sum += image[address];
  return (u8)(0x100 - sum);
}

// EEPROMアドレスの名前（HSR-8498HB。"HSR Serial Communication" 2007-09-23 より）
const char *eepromName(u8 address)
{
  switch (address)
  {
  case 0x00: return "P-gain set1 (H)";
  case 0x01: return "P-gain set1 (L)";
  case 0x02: return "Dead-zone set1";
  case 0x03: return "D-gain set1";
  case 0x04: return "D-gain? set1";
  case 0x06: return "Speed";
  case 0x07: return "Center position (H)?";
  case 0x08: return "Center position (L)?";
  case 0x09: return "Min position limit (H)";
  case 0x0A: return "Min position limit (L)";
  case 0x0B: return "Max position limit (H)";
  case 0x0C: return "Max position limit (L)";
  case 0x13: return "Left limit?";
  case 0x14: return "Right limit?";
  case 0x1B: return "Direction (0x29 fwd, 0x28 rev)?";
  case 0x1F: return "P-gain set2 (H)";
  case 0x20: return "P-gain set2 (L)";
  case 0x21: return "Dead-zone set2";
  case 0x22: return "D-gain set2";
  case 0x23: return "D-gain? set2";
  case 0x24: return "P-gain set3 (H)";
  case 0x25: return "P-gain set3 (L)";
  case 0x26: return "Dead-zone set3";
  case 0x27: return "D-gain set3";
  case 0x28: return "D-gain? set3";
  case EEPROM_ADDR_ID: return "Servo ID";
  case EEPROM_ADDR_CHECKSUM: return "EEPROM checksum";
  default: return "Unknown";
  }
}

// EEPROMを安全に書き換え（現在のチェックサム確認 → データ書き込み → チェックサム更新 → 読み返し）
// address から length バイト（1 or 2, 2バイトは上位が先）を書き込む。1台のみ接続で使うこと
bool writeEepromSafely(u8 address, const u8 *data, u8 length)
{
  u8 image[EEPROM_ADDR_CHECKSUM + 1];
  Serial.println("Reading EEPROM 0x00-0x2C...");
  if (!readEepromImage(image))
    return false;
  if (image[EEPROM_ADDR_CHECKSUM] != eepromChecksum(image))
  {
    Serial.println("Error: Current EEPROM checksum is invalid. Aborted to avoid breaking the servo.");
    return false;
  }

  for (u8 i = 0; i < length; i++)
    image[address + i] = data[i];
  u8 checksum = eepromChecksum(image);

  for (u8 i = 0; i < length; i++)
  {
    if (!hmiWriteEeprom(address + i, data[i]))
    {
      Serial.print("Error: Failed to write EEPROM 0x");
      Serial.print(address + i, HEX);
      Serial.println(i == 0 ? " (EEPROM unchanged)." : ". Run the command again immediately.");
      return false;
    }
  }
  if (!hmiWriteEeprom(EEPROM_ADDR_CHECKSUM, checksum))
  {
    Serial.println("[Error] Data was written but the checksum write failed. Run the command again immediately.");
    return false;
  }

  // 読み返して確認
  for (u8 i = 0; i < length; i++)
  {
    u8 value;
    if (hmiReadEeprom(address + i, &value) != HMI_OK || value != data[i])
    {
      Serial.println("[Error] Verification failed.");
      return false;
    }
  }
  u8 readChecksum;
  if (hmiReadEeprom(EEPROM_ADDR_CHECKSUM, &readChecksum) != HMI_OK || readChecksum != checksum)
  {
    Serial.println("[Error] Checksum verification failed.");
    return false;
  }

  Serial.print("Verification: data and checksum (0x");
  Serial.print(checksum, HEX);
  Serial.println(") [OK]");
  return true;
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
  bool selected[HMI_ID_MAX + 1] = {false};
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
    if (first < HMI_ID_MIN || last > HMI_ID_MAX || first > last)
      return -1;

    for (int id = first; id <= last; id++)
      selected[id] = true;
  }

  int count = 0;
  for (int id = HMI_ID_MIN; id <= HMI_ID_MAX; id++)
  {
    if (selected[id])
      out[count++] = id;
  }
  return count > 0 ? count : -1;
}

// 一斉制御リストの全サーボに同じ位置を送信
void moveSyncServos(u16 pulseUs)
{
  for (u8 i = 0; i < syncServoCount; i++)
    hmiSetPosition(syncServoIDs[i], pulseUs);

  Serial.print("Position: ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.print("), ");
  Serial.print(pulseUs);
  Serial.print("us (");
  Serial.print(pulseToAngle(pulseUs), 1);
  Serial.println("deg)");
}

// ===== コマンド =====

// [f]コマンド: サーボ認識（接続中の1台のバージョンとIDを読み取り）
void executeServoDetectionCommand()
{
  Serial.println("Reading version and ID (HMI read commands have no ID: connect only ONE servo)...");

  u8 version, id;
  HmiResult result = hmiReadVersionId(&version, &id);
  if (result != HMI_OK)
  {
    Serial.print("Error: ");
    Serial.println(hmiResultText(result));
    Serial.println("Hint: Use [t] to check TX, [g] to see raw received bytes, [u] to switch signal polarity");
    return;
  }

  Serial.print("Found servo: version 0x");
  Serial.print(version, HEX);
  Serial.print(", ID ");
  Serial.println(id);

  if (id > HMI_ID_MAX)
  {
    Serial.println("[Warning] ID is out of range (0-127). Reply may be corrupted.");
    return;
  }

  targetServoID = id;
  syncServoIDs[0] = id;
  syncServoCount = 1;
  Serial.print("Target ID and sync servo IDs auto-set to: ");
  Serial.println(id);
}

// [z]コマンド: ターゲットID変更
void executeChangeTargetIdCommand()
{
  Serial.print("Current target ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter new target ID (0-127): ");

  long newId;
  if (!parseNumber(readInput(), &newId) || newId < HMI_ID_MIN || newId > HMI_ID_MAX)
  {
    Serial.println("Error: ID out of range (0-127)");
    return;
  }

  targetServoID = newId;
  Serial.print("Target ID changed to: ");
  Serial.println(targetServoID);
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
  {
    Serial.println("Error: Invalid ID list (valid IDs: 0-127)");
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

// [r]コマンド: サーボ位置読み取り（0xE9: 速度設定も同時に送られる）
void executeReadPositionCommand()
{
  Serial.print("Reading position from servo ID ");
  Serial.print(targetServoID);
  Serial.print(" (speed 0x");
  Serial.print(servoSpeed, HEX);
  Serial.println(" is also sent)...");

  u16 pulseUs;
  HmiResult result = hmiSetSpeedReadPosition(targetServoID, servoSpeed, &pulseUs);
  if (result != HMI_OK)
  {
    Serial.print("Error: ");
    Serial.println(hmiResultText(result));
    return;
  }

  Serial.print("Position: ");
  Serial.print(pulseUs);
  Serial.print("us (");
  Serial.print(pulseToAngle(pulseUs), 1);
  Serial.println("deg)");
}

// [v]コマンド: 電圧・駆動パルス幅読み取り（0xE8, 宛先IDなし）
void executeReadVoltageCommand()
{
  Serial.println("Reading voltage and motor pulse width (no ID: connect only ONE servo)...");

  u8 pulseWidth, voltage;
  HmiResult result = hmiTransaction(HMI_CMD_PULSE_VOLT, 0x00, 0x00, &pulseWidth, &voltage);
  if (result != HMI_OK)
  {
    Serial.print("Error: ");
    Serial.println(hmiResultText(result));
    return;
  }

  Serial.print("Voltage    : ");
  Serial.print(voltage);
  Serial.print(" (");
  Serial.print(voltage * HMI_VOLTAGE_SCALE, 2);
  Serial.println("V)");
  Serial.print("Pulse width: ");
  Serial.print(pulseWidth);
  Serial.println(" (motor drive, raw 0-255)");
}

// [m]コマンド: バージョンとID読み取り
void executeReadVersionCommand()
{
  Serial.println("Reading version and ID (connect only ONE servo)...");

  u8 version, id;
  HmiResult result = hmiReadVersionId(&version, &id);
  if (result != HMI_OK)
  {
    Serial.print("Error: ");
    Serial.println(hmiResultText(result));
    return;
  }

  Serial.println("Protocol        : Hitec HMI (serial mode)");
  Serial.print("Version         : 0x");
  Serial.println(version, HEX);
  Serial.print("Servo ID        : ");
  Serial.println(id);
}

// [i]コマンド: サーボID変更（EEPROM 0x29 書き込み + チェックサム 0x2C 更新。1台のみ接続）
void executeChangeServoIdCommand()
{
  Serial.println("[Warning] Connect only ONE servo. Do not power off during writing.");

  // 現在のIDを確認
  u8 version, currentId;
  HmiResult result = hmiReadVersionId(&version, &currentId);
  if (result != HMI_OK)
  {
    Serial.print("Error: Cannot read current ID (");
    Serial.print(hmiResultText(result));
    Serial.println(")");
    return;
  }

  // EEPROMのIDが0xE7で読んだIDと一致するか確認
  u8 eepromId;
  if (hmiReadEeprom(EEPROM_ADDR_ID, &eepromId) != HMI_OK || eepromId != currentId)
  {
    Serial.println("Error: EEPROM ID does not match the ID read by 0xE7. Aborted.");
    return;
  }

  Serial.print("Current servo ID: ");
  Serial.println(currentId);
  Serial.println("Enter new servo ID (0-127): ");

  long newId;
  if (!parseNumber(readInput(), &newId) || newId < HMI_ID_MIN || newId > HMI_ID_MAX)
  {
    Serial.println("Error: ID out of range (0-127)");
    return;
  }
  if (newId == currentId)
  {
    Serial.println("New ID is the same as current ID. Nothing to do.");
    return;
  }

  // ID書き込み → チェックサム更新（両方そろわないとモータが動かなくなる）
  Serial.print("Changing servo ID ");
  Serial.print(currentId);
  Serial.print(" -> ");
  Serial.println(newId);

  u8 data[1] = {(u8)newId};
  if (!writeEepromSafely(EEPROM_ADDR_ID, data, 1))
    return;

  targetServoID = newId;
  syncServoIDs[0] = newId;
  syncServoCount = 1;
  Serial.print("Target ID and sync servo IDs set to: ");
  Serial.println(newId);

  // 新IDが反映されているか確認（反映されていなければ電源の入れ直しが必要）
  if (hmiReadVersionId(&version, &currentId) == HMI_OK && currentId == newId)
    Serial.println("New ID is active.");
  else
    Serial.println("New ID is saved. Power-cycle the servo to apply it.");
}

// [c]コマンド: 全サーボ位置を中央にリセット
void executeResetAllPositionsCommand()
{
  Serial.print("Resetting ");
  Serial.print(syncServoCount);
  Serial.println(" servos to center (1500us)...");
  moveSyncServos(HMI_PULSE_CENTER);
}

// [x]コマンド: 全サーボに角度を一斉送信
void executeAngleCommand()
{
  Serial.println("Enter angle (-95 to 95 degrees, 1 deg = 10us): ");

  String input = readInput();
  float angle = input.toFloat();

  // 角度範囲チェック
  if (input.length() == 0 || angle < -95.0 || angle > 95.0)
  {
    Serial.println("Error: Angle out of range (-95 to 95)");
    return;
  }

  moveSyncServos(angleToPulse(angle));
  Serial.println("Angle command sent successfully");
}

// 返信が 03 06 のコマンド（0xEA/0xEB/0xEF）を送り、結果を表示
void sendAckCommand(u8 command, u8 param1, u8 param2)
{
  u8 reply1, reply2;
  HmiResult result = hmiTransaction(command, param1, param2, &reply1, &reply2);
  if (result != HMI_OK)
  {
    Serial.print("Sent. Reply: ");
    Serial.println(hmiResultText(result));
    return;
  }
  Serial.print("Reply: 0x");
  Serial.print(reply1, HEX);
  Serial.print(" 0x");
  Serial.print(reply2, HEX);
  Serial.println((reply1 == 0x03 && reply2 == 0x06) ? " [OK]" : " (expected 03 06)");
}

// [d]コマンド: 脱力（0xEF, 全サーボ。手で動かせるようになる）
void executeReleaseCommand()
{
  Serial.println("Release (0xEF): position control of ALL servos is stopped.");
  sendAckCommand(HMI_CMD_RELEASE, 0x00, 0x00);
  Serial.println("Note: Any move command re-enables ALL servos. Use [e] to hold at the current position.");
}

// [e]コマンド: 脱力から復帰（現在位置を読み、その位置を目標にして保持）
void executeHoldCommand()
{
  Serial.print("Hold at current position: ");
  Serial.print(syncServoCount);
  Serial.print(" servos (IDs: ");
  printSyncServoIDs();
  Serial.println(")...");

  u16 positions[MAX_SYNC_SERVOS];
  for (u8 i = 0; i < syncServoCount; i++)
  {
    u8 id = syncServoIDs[i];
    if (hmiSetSpeedReadPosition(id, servoSpeed, &positions[i]) != HMI_OK ||
        positions[i] < HMI_PULSE_MIN || positions[i] > HMI_PULSE_MAX)
    {
      Serial.print("Error: Cannot read position of ID ");
      Serial.print(id);
      Serial.println(". Aborted (no command sent).");
      return;
    }
  }

  // 1台のときは、手で動かした後の急な動きを防ぐため軌道目標値（RAM 0x06-0x07）を現在位置に合わせる
  if (syncServoCount == 1)
  {
    hmiTransaction(HMI_CMD_WRITE_MEMORY, MEMORY_ADDR_SETPOINT_LOW, positions[0] & 0xFF, NULL, NULL);
    hmiTransaction(HMI_CMD_WRITE_MEMORY, MEMORY_ADDR_SETPOINT_HIGH, positions[0] >> 8, NULL, NULL);
  }
  else
  {
    Serial.println("[Warning] With multiple servos, a servo moved by hand may jump toward its old target.");
  }

  for (u8 i = 0; i < syncServoCount; i++)
  {
    hmiSetPosition(syncServoIDs[i], positions[i]);
    Serial.print("ID ");
    Serial.print(syncServoIDs[i]);
    Serial.print(": hold at ");
    Serial.print(positions[i]);
    Serial.print("us (");
    Serial.print(pulseToAngle(positions[i]), 1);
    Serial.println("deg)");
  }
}

// [y]コマンド: Go/Stop（0xEB, 宛先IDなしで全サーボに届く）
void executeGoStopCommand()
{
  Serial.println("Select: [1] GO  [0] STOP (suspend target position changes, torque stays ON)");
  String input = readInput();
  if (input != "0" && input != "1")
  {
    Serial.println("Error: Invalid selection");
    return;
  }
  bool go = (input == "1");

  Serial.print(go ? "Motor GO" : "Motor STOP");
  Serial.println(" (0xEB, sent to all servos)...");
  if (!go)
    Serial.println("Note: STOP suspends target position changes. It does NOT release the torque.");

  sendAckCommand(HMI_CMD_GO_STOP, go ? 1 : 0, 0x00);
}

// [j]コマンド: 制御パラメータセット選択（0xEA, 全サーボ。電源投入時はセット1）
void executeParamSetCommand()
{
  Serial.println("Select control parameter set (P-gain, D-gain, dead-band in EEPROM): [1] [2] [3]");
  String input = readInput();
  if (input != "1" && input != "2" && input != "3")
  {
    Serial.println("Error: Invalid selection");
    return;
  }
  Serial.print("Parameter set ");
  Serial.print(input);
  Serial.println(" (0xEA, sent to all servos)...");
  sendAckCommand(HMI_CMD_PARAM_SET, 0x00, input.toInt());
}

// [p]コマンド: EEPROM読み出し（0xE1, 宛先IDなし: 1台のみ接続）
void executeReadEepromCommand()
{
  Serial.println("Enter EEPROM address (0x00-0xFF, or 'all' for 0x00-0x2C). Connect only ONE servo: ");
  String input = readInput();

  if (input == "all")
  {
    u8 image[EEPROM_ADDR_CHECKSUM + 1];
    if (!readEepromImage(image))
      return;
    for (u8 address = 0; address <= EEPROM_ADDR_CHECKSUM; address++)
    {
      char line[64];
      snprintf(line, sizeof(line), "0x%02X: 0x%02X (%3d)  %s", address, image[address], image[address], eepromName(address));
      Serial.println(line);
    }
    Serial.print("Checksum: ");
    Serial.println(image[EEPROM_ADDR_CHECKSUM] == eepromChecksum(image) ? "valid [OK]" : "INVALID (motor will not move)");
    return;
  }

  long address;
  if (!parseNumber(input, &address) || address < 0 || address > 0xFF)
  {
    Serial.println("Error: Address out of range (0x00-0xFF)");
    return;
  }

  u8 value;
  HmiResult result = hmiReadEeprom(address, &value);
  if (result != HMI_OK)
  {
    Serial.print("Error: ");
    Serial.println(hmiResultText(result));
    return;
  }
  char line[64];
  snprintf(line, sizeof(line), "EEPROM 0x%02X: 0x%02X (%d)  %s", (u8)address, value, value, eepromName(address));
  Serial.println(line);
}

// [s]コマンド: EEPROM書き込み（1 or 2バイト + チェックサム自動更新, 1台のみ接続）
void executeWriteEepromCommand()
{
  Serial.println("[Warning] Connect only ONE servo. Do not power off during writing.");
  Serial.println("Enter EEPROM address (0x00-0x2B, except 0x29 = ID: use [i]): ");

  long address;
  if (!parseNumber(readInput(), &address) || address < 0 || address >= EEPROM_ADDR_CHECKSUM || address == EEPROM_ADDR_ID)
  {
    Serial.println("Error: Address out of range");
    return;
  }

  Serial.println("Enter data length (1 or 2 bytes, 2 bytes = high byte first): ");
  long length;
  if (!parseNumber(readInput(), &length) || (length != 1 && length != 2) ||
      address + length - 1 >= EEPROM_ADDR_CHECKSUM ||
      (address <= EEPROM_ADDR_ID && address + length - 1 >= EEPROM_ADDR_ID))
  {
    Serial.println("Error: Invalid length (must not include 0x29 or 0x2C)");
    return;
  }

  Serial.print("Enter value (");
  Serial.print(length == 1 ? "0-255" : "0-65535");
  Serial.println(", decimal or 0x hex): ");
  long value;
  if (!parseNumber(readInput(), &value) || value < 0 || value > (length == 1 ? 0xFF : 0xFFFF))
  {
    Serial.println("Error: Value out of range");
    return;
  }

  u8 data[2];
  if (length == 1)
    data[0] = value;
  else
  {
    data[0] = value >> 8;
    data[1] = value & 0xFF;
  }

  Serial.print("Writing 0x");
  Serial.print(value, HEX);
  Serial.print(" to EEPROM 0x");
  Serial.print(address, HEX);
  Serial.print(" (");
  Serial.print(eepromName(address));
  Serial.println(")");

  if (writeEepromSafely(address, data, length))
    Serial.println("Saved. Power-cycle the servo to apply EEPROM changes.");
}

// [k]コマンド: 速度設定（0xE9。応答で現在位置も表示）
void executeSetSpeedCommand()
{
  Serial.print("Current speed: 0x");
  Serial.println(servoSpeed, HEX);
  Serial.println("Enter speed for target servo (1-255 or 0x01-0xFF. Larger is faster): ");

  long value;
  if (!parseNumber(readInput(), &value) || value < 1 || value > 255)
  {
    Serial.println("Error: Value out of range (1-255)");
    return;
  }

  servoSpeed = value;
  u16 pulseUs;
  HmiResult result = hmiSetSpeedReadPosition(targetServoID, servoSpeed, &pulseUs);

  Serial.print("Speed set to 0x");
  Serial.print(servoSpeed, HEX);
  if (result == HMI_OK)
  {
    Serial.print(". Position: ");
    Serial.print(pulseUs);
    Serial.println("us");
  }
  else
  {
    Serial.print(" (sent, reply: ");
    Serial.print(hmiResultText(result));
    Serial.println(")");
  }
}

// [t]コマンド: 送信テスト（ID0-127全員に位置を送り往復。受信不要でTX側の確認用）
void executeTxTestCommand()
{
  Serial.println("TX test: sending position to ALL IDs (0-127): +30, -30, +30, -30, center.");
  Serial.println("No reception is needed. If the servo moves, TX (ESP32 -> servo) is OK.");

  const float angles[] = {30.0, -30.0, 30.0, -30.0, 0.0};
  for (u8 step = 0; step < sizeof(angles) / sizeof(angles[0]); step++)
  {
    u16 pulseUs = angleToPulse(angles[step]);
    for (int id = HMI_ID_MIN; id <= HMI_ID_MAX; id++)
      hmiSetPosition(id, pulseUs);
    Serial.print("  ");
    Serial.print(angles[step], 0);
    Serial.println("deg sent");
    delay(700);
  }

  Serial.println("TX test finished.");
  Serial.println("Moved     -> TX OK. If reads fail, check pull-up / polarity (use [g], [u]).");
  Serial.println("Not moved -> check power, wiring, and signal polarity ([u]). Power-cycle the servo.");
}

// [g]コマンド: 受信テスト（0xE7を送り、受信した生データを16進表示）
void executeRxDumpCommand()
{
  u8 packet[HMI_SEND_SIZE];
  buildHmiPacket(packet, HMI_CMD_VERSION_ID, 0x00, 0x00);

  u8 raw[16], rawCount = 0, reply1, reply2;
  HmiResult result = hmiTransaction(HMI_CMD_VERSION_ID, 0x00, 0x00, &reply1, &reply2, raw, &rawCount);

  Serial.print("Reply mode: ");
  Serial.println(replyFramed ? "framed (aligned to UART start bits)" : "release only");
  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());
  Serial.print("Sent (5 bytes + 2 reply slots): ");
  for (u8 i = 0; i < HMI_SEND_SIZE; i++)
  {
    if (packet[i] < 0x10)
      Serial.print("0");
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println("[00 00]");

  Serial.print("Received ");
  Serial.print(rawCount);
  Serial.print(" byte(s): ");
  for (u8 i = 0; i < rawCount; i++)
  {
    if (raw[i] < 0x10)
      Serial.print("0");
    Serial.print(raw[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // 結果の解釈
  if (rawCount == 0)
    Serial.println("-> Nothing received. Check pull-up on the signal line, or try [u] (polarity).");
  else if (rawCount == HMI_SEND_SIZE && memcmp(raw, packet, HMI_SEND_SIZE) == 0)
    Serial.println("-> Only the echo of our own packet. Servo did not write the reply bytes.");
  else if (result == HMI_OK)
  {
    Serial.print("-> Reply: version 0x");
    Serial.print(reply1, HEX);
    Serial.print(", ID ");
    Serial.println(reply2);
  }
  else
    Serial.println("-> Unexpected data. Possible polarity / pull-up / noise problem.");
}

// [w]コマンド: 返信の受け方の切り替え（診断用）
void executeToggleReplyModeCommand()
{
  replyFramed = !replyFramed;
  Serial.print("Reply mode: ");
  Serial.println(replyFramed ? "framed (default, aligned to UART start bits)" : "release only (pull-up keeps the line)");
  Serial.println("Note: position commands ([x] [c] [t] [e]) are always sent in release-only mode.");
}

// [u]コマンド: 信号極性の切り替え（診断用）
void executeTogglePolarityCommand()
{
  signalInverted = !signalInverted;
  beginServoSerial(true);
  Serial.print("Signal polarity: ");
  Serial.println(signalInverted ? "Inverted (HMI spec: idle LOW, start bit HIGH)" : "Normal (idle HIGH)");
  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());
}

void disp_menu()
{
  Serial.println("============================================");
  Serial.print("Current target ID: ");
  Serial.print(targetServoID);
  Serial.print(", Sync IDs: ");
  printSyncServoIDs();
  Serial.print(", Speed: 0x");
  Serial.print(servoSpeed, HEX);
  Serial.print(", Signal: ");
  Serial.print(signalInverted ? "inverted" : "normal");
  Serial.print(", Baud rate: ");
  Serial.print(HMI_BAUD_RATE);
  Serial.println(" bps 8N2 (fixed), Protocol: Hitec HMI");
  Serial.println("Menu: [f] Find servo    [n] Set sync servo IDs   [z] Set target ID ");
  Serial.println("      [v] Read voltage  [r] Read position        [m] Read version");
  Serial.println("      [c] All to center [x] Send angle to all    [k] Set speed");
  Serial.println("      [e] Hold position [d] Release (0xEF)       [y] Go/Stop (0xEB)");
  Serial.println("      [p] Read EEPROM   [s] Write EEPROM         [i] Change servo ID");
  Serial.println("      [j] Param set     (0xEA: P/D-gain, dead-band set 1-3, all servos)");
  Serial.println("      [t] TX test       [g] RX dump (debug)      [u] Signal polarity");
  Serial.println("      [w] Reply mode (framed / release only, debug)");
  Serial.println("Note: [f] [m] [v] [p] [s] [i] have no ID. Connect only ONE servo for them.");
  Serial.println("      [h] Change servo type (restart)");
  Serial.println("Input Command:");
}

// 初期化
void setup()
{
  Serial.begin(115200);

  // サーボバス開始（ENピン設定・エコー判定を含む）
  beginServoSerial(false);

  delay(1000);
  Serial.print("Servo bus echo: ");
  Serial.println(busEchoText());

  Serial.println("=====================================");
  Serial.println("Hitec HSR Servo Setter");
  Serial.println("HMI serial mode (HSR-8498HB, 19200 bps 8N2)");
  Serial.println("=====================================");
  Serial.println("Use [f] to read the connected servo first.");

  disp_menu();
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
    else if (command == "c")
    {
      executeResetAllPositionsCommand();
    }
    else if (command == "f")
    {
      executeServoDetectionCommand();
    }
    else if (command == "m")
    {
      executeReadVersionCommand();
    }
    else if (command == "e")
    {
      executeHoldCommand();
    }
    else if (command == "d")
    {
      executeReleaseCommand();
    }
    else if (command == "y")
    {
      executeGoStopCommand();
    }
    else if (command == "p")
    {
      executeReadEepromCommand();
    }
    else if (command == "s")
    {
      executeWriteEepromCommand();
    }
    else if (command == "j")
    {
      executeParamSetCommand();
    }
    else if (command == "i")
    {
      executeChangeServoIdCommand();
    }
    else if (command == "k")
    {
      executeSetSpeedCommand();
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
      executeToggleReplyModeCommand();
    }
    else if (command == "u")
    {
      executeTogglePolarityCommand();
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

// 読み取りコマンドだけで HMI のサーボを探す（サーボは動かない）。
// サーボがいなくても線の状態がそのまま0x00/0xFFとして読めるため、
// バージョンとIDの読み出し（0xE7）で、0x00/0xFF以外の同じ値が2回続けて返ったときだけ見つかったとする
bool detect()
{
  beginServoSerial(false);

  // 他の種類を調べている間、線は非反転（アイドルHIGH）だった。反転のアイドル（LOW）で
  // サーボの受信を落ち着かせてから読む（setupと同じ待ち）
  delay(1000);

  u8 lastVersion = 0, lastId = 0;
  HmiResult lastResult = HMI_NO_RESPONSE;
  bool matched = false;
  for (int attempt = 0; attempt < 4 && !matched; attempt++)
  {
    u8 version = 0, id = 0;
    HmiResult result = hmiReadVersionId(&version, &id);
    bool valid = (result == HMI_OK && version != 0x00 && version != 0xFF && id <= HMI_ID_MAX);
    matched = valid && lastResult == HMI_OK && version == lastVersion && id == lastId;
    lastResult = valid ? HMI_OK : HMI_INVALID_RESPONSE;
    lastVersion = version;
    lastId = id;
    delay(50);
  }

  // 見つからなかったときは、最後に読めた値を表示（原因調べ用）
  if (!matched)
  {
    Serial.print("(last 0xE7 reply: version 0x");
    Serial.print(lastVersion, HEX);
    Serial.print(", ID ");
    Serial.print(lastId);
    Serial.print(") ");
  }
  return matched;
}
} // namespace hitec
