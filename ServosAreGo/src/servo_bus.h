// 半二重サーボバス（Meridian_LITE の MrdHalfDuplexUart を簡略化して移植。各 Servo Setter 共通）
//
// - 配線は2通り（起動メニューの [w] で選び、NVSに保存。busDirectWiring）
//     ① 半二重回路あり（Meridian Board 等）: EN(DIR)ピンで送受信を切り替える。エコーの有無は自動判定
//          SN74LXC1T45版（TXとRXが同じ端子に直結）: エコーあり → 受信前に照合して読み捨てる
//          3ステートバッファ版                     : エコーなし
//     ② 半二重回路なし（RXとTXを両方サーボの信号線につなぐ）: ENピンは使わない。
//          常にエコーありとして扱い、受信中はTXの出力を切ってRXの内部プルアップで線をHIGHに保つ
// - エコーありの回路では、受信中にTXピンの出力を切って信号線上のぶつかりを防ぐ
// - サーボは送信完了の直後に返信するため、送信完了の確認と受信への切り替えを
//   IRAM上で割り込みを止め、レジスタを直接操作して行う（返信の先頭欠けを防ぐ）
//
// 使い方:
//   busBegin()        : シリアル開始とエコー判定
//   busDirTx/busDirRx : 送受信方向の切り替え（PIN_ENを直接操作せず、必ずこちらを使う）
//   busEndSendToRx()  : write()の後、送信完了を待ってすぐ受信方向にする
//   busSkipEcho()     : 受信の前に送信エコーを照合して読み捨てる（エコーなしなら何もしない）

#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <hal/uart_ll.h>

extern bool busDirectWiring; // true: ② 半二重回路なし（main.cpp で起動時に読み込む）

static HardwareSerial *busSerial = nullptr;                 // 使用するUART
static uart_dev_t *busUartHw = nullptr;                     // 送信完了をレジスタで確認するためのUART
static portMUX_TYPE busMux = portMUX_INITIALIZER_UNLOCKED;  // 切り替え時の割り込み停止用
static int8_t busPinDir = -1;                               // EN(DIR)ピン（HIGH: 送信, LOW: 受信, -1: 使わない）
static int8_t busTxReleasePin = -1;                         // 受信中に出力を切るTXピン（-1: 切らない）
static bool busEchoEnabled = false;                         // 送信エコーを照合して読み捨てるか

// TXピンの出力ON/OFF（UARTとピンの対応付けは変更しない）
__attribute__((always_inline)) static inline void busTxOutput(int8_t pin, bool on)
{
  if (pin < 0)
    return;
  if (pin < 32)
  {
    if (on)
      GPIO.enable_w1ts = (1u << pin);
    else
      GPIO.enable_w1tc = (1u << pin);
  }
  else
  {
    if (on)
      GPIO.enable1_w1ts.val = (1u << (pin - 32));
    else
      GPIO.enable1_w1tc.val = (1u << (pin - 32));
  }
}

// 送信完了を待ってENをLOW（受信）にする（IRAM上で割り込みを止めて実行）
static void __attribute__((noinline)) IRAM_ATTR busWaitTxAndDirRx(uart_dev_t *hw, int8_t pinDir, int8_t pinTxRelease,
                                                                  portMUX_TYPE *mux)
{
  portENTER_CRITICAL(mux);
  while (!uart_ll_is_tx_idle(hw))
  {
  }
  busTxOutput(pinTxRelease, false); // 先にTXを切ってから受信方向にする
  if (pinDir >= 0)
    gpio_ll_set_level(&GPIO, (gpio_num_t)pinDir, 0); // LOW: 受信
  portEXIT_CRITICAL(mux);
}

// 送信方向に切り替え（先にENを送信にしてからTXを出力）
static inline void busDirTx()
{
  if (busPinDir >= 0)
    digitalWrite(busPinDir, HIGH);
  busTxOutput(busTxReleasePin, true);
}

// 受信方向に切り替え（先にTXを切ってからENを受信に）
static inline void busDirRx()
{
  busTxOutput(busTxReleasePin, false);
  if (busPinDir >= 0)
    digitalWrite(busPinDir, LOW);
}

// 受信バッファを空にする
static inline void busFlushRx()
{
  while (busSerial->available() > 0)
    busSerial->read();
}

// write()の後に呼ぶ: 送信完了を待ってすぐ受信方向にする
static inline void busEndSendToRx()
{
  busWaitTxAndDirRx(busUartHw, busPinDir, busTxReleasePin, &busMux);
}

// 送信エコーを照合して読み捨てる（エコーなしの回路では何もしない）。一致すればtrue
static bool busSkipEcho(const uint8_t *tx, size_t length, unsigned long timeoutMs)
{
  if (!busEchoEnabled)
    return true;

  unsigned long startTime = millis();
  size_t index = 0;
  while (index < length)
  {
    if (busSerial->available() > 0)
    {
      if ((uint8_t)busSerial->read() != tx[index])
        return false;
      index++;
    }
    else if (millis() - startTime >= timeoutMs)
    {
      return false;
    }
  }
  return true;
}

// 送信エコーの有無を判定（0x55: どのサーボのプロトコルでもコマンドと見なされない）
static bool busDetectEcho()
{
  bool echo = true;
  for (int i = 0; i < 3 && echo; i++)
  {
    busFlushRx();
    busDirTx();
    busSerial->write((uint8_t)0x55);
    busSerial->flush();
    busDirRx();
    uint8_t value;
    echo = (busSerial->readBytes(&value, 1) == 1 && value == 0x55);
  }
  busFlushRx();
  return echo;
}

static bool busEchoMissing = false; // ② なのにエコーが返らなかった（RXかTXが信号線につながっていない）

// シリアル開始とエコー判定（再設定にも使える）。idleTx: 判定後の待機方向（true: 送信, false: 受信）
// ② 半二重回路なしでは pinDir を使わない
static void busBegin(HardwareSerial *serial, int8_t pinRx, int8_t pinTx, int8_t pinDir, unsigned long baud,
                     uint32_t config, bool invert = false, bool idleTx = true)
{
  busSerial = serial;
  busPinDir = busDirectWiring ? -1 : pinDir;
  busTxReleasePin = -1;
  if (busPinDir >= 0)
  {
    pinMode(busPinDir, OUTPUT);
    digitalWrite(busPinDir, LOW);
  }

  // rxfifo_full_thrhd = 1, RXタイムアウト1シンボル: 受信したバイトをすぐ読めるようにする
  serial->begin(baud, config, pinRx, pinTx, invert, 20000UL, 1);
  serial->setRxTimeout(1);
  serial->setTimeout(20);
  busUartHw = UART_LL_GET_HW(serial == &Serial1 ? 1 : (serial == &Serial2 ? 2 : 0));

  busEchoEnabled = busDetectEcho();
  busEchoMissing = busDirectWiring && !busEchoEnabled;
  if (busDirectWiring)
    busEchoEnabled = true; // RXとTXが同じ信号線なので、エコーは必ずある
  if (busEchoEnabled)
  {
    // TXとA端子（② では信号線）が直結: 受信中はTXを切り、信号線が浮かないようRXをプルアップ
    busTxReleasePin = pinTx;
    gpio_pullup_en((gpio_num_t)pinRx);
    GPIO.func_out_sel_cfg[pinTx].oen_sel = 1; // TXの出力有効をGPIOレジスタ側で制御
  }

  if (idleTx)
    busDirTx();
  else
    busDirRx();
}

// 判定結果の説明（起動ログ・メニュー表示用）
static inline const char *busEchoText()
{
  if (busEchoMissing)
    return "direct wiring, but NO echo! Connect both RX and TX to the servo signal line";
  if (busDirectWiring)
    return "direct wiring (RX+TX on signal line, TX released while receiving)";
  return busEchoEnabled ? "detected (SN74LXC1T45 type, TX released while receiving)" : "none (3-state buffer type)";
}
