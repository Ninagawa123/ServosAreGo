// ServosAreGo: 各社シリアルサーボの設定ツール（Feetech / Dynamixel / Futaba / JR XBUS / Hitec / Kondo / PWM）
//
// 起動するとサーボ種類の選択メニューを表示し、選んだ種類の Servo Setter を動かす。
// [0] を選ぶと、つながっているサーボの種類と通信速度を自動判別する。
// シリアルサーボが見つからなければ、確認したうえでPWMで動かし、動いたかを人に確かめてもらう。
// 選んだ種類はNVSに保存し、次回は入力がなければ同じ種類で起動する。
// 動作中は [h] でサーボ種類を選び直せる（選び直すと再起動する）。

#include <Arduino.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include "servo_select.h"

#define SELECT_TIMEOUT 5000          // 前回の種類で自動起動するまでの待ち(ms)
#define BOOT_MAGIC 0x53415247UL      // 再起動後の動作（bootAction）が有効である目印
#define NVS_KEY_KIND "type"         // 種類の保存キー（並びを変えたら名前も変え、古い番号を使わない）
#define KIND_DETECT -2               // askServoKind() の戻り値: [0] 自動判別

// サーボ種類（SERVO_KINDS の並び）
enum
{
  KIND_PWM,
  KIND_FEETECH_STS,
  KIND_FEETECH_SCS,
  KIND_DYNAMIXEL,
  KIND_FUTABA,
  KIND_JR_XBUS,
  KIND_HITEC,
  KIND_KONDO,
};

// サーボ種類の一覧（[1] がPWM。Feetech は STS と SCS で位置の範囲とバイト順が違うため分ける）
struct ServoKind
{
  const char *name;
  void (*setup)();
  void (*loop)();
  void (*presetBaud)(unsigned long baud); // 通信速度が固定の種類はNULL
};

const ServoKind SERVO_KINDS[] = {
    {"PWM servo (50 Hz, no reply)", pwm::setup, pwm::loop, NULL},
    {"Feetech STS (STS/SMS series)", [] { feetech::setup(0); }, feetech::loop, feetech::presetBaud},
    {"Feetech SCS (SCS series)", [] { feetech::setup(1); }, feetech::loop, feetech::presetBaud},
    {"Dynamixel (Protocol 2.0)", dynamixel::setup, dynamixel::loop, dynamixel::presetBaud},
    {"Futaba TTL (RS30x)", futaba::setup, futaba::loop, futaba::presetBaud},
    {"JR XBUS", jrxbus::setup, jrxbus::loop, NULL},
    {"Hitec HSR (HMI)", hitec::setup, hitec::loop, NULL},
    {"Kondo KRS (ICS3.5/3.6)", kondo::setup, kondo::loop, kondo::presetBaud},
};
#define SERVO_KIND_COUNT (sizeof(SERVO_KINDS) / sizeof(SERVO_KINDS[0]))

// 自動判別の手順。kind に見つかった種類を入れ、見つかった通信速度を返す（なければ0）
// 位置指令にチェックサムがない ICS（近藤）を最初に調べ、他のプロトコルの送信で誤って動かないようにする。
// 次に各種類を工場出荷時の速度で調べ、最後にそれ以外の速度を調べる
struct DetectStep
{
  const char *label;
  unsigned long (*probe)(int *kind);
};

const DetectStep DETECT_STEPS[] = {
    {"Kondo KRS        115200 bps",
     [](int *kind) { *kind = KIND_KONDO; return kondo::detect(true); }},
    {"Kondo KRS        other baud rates",
     [](int *kind) { *kind = KIND_KONDO; return kondo::detect(false); }},
    {"Feetech STS/SCS  1000000 bps",
     [](int *kind) {
       uint8_t series;
       unsigned long baud = feetech::detect(true, &series);
       *kind = series ? KIND_FEETECH_SCS : KIND_FEETECH_STS;
       return baud;
     }},
    {"Dynamixel        57600 bps",
     [](int *kind) { *kind = KIND_DYNAMIXEL; return dynamixel::detect(true); }},
    {"Futaba TTL       115200 bps",
     [](int *kind) { *kind = KIND_FUTABA; return futaba::detect(true); }},
    {"JR XBUS          250000 bps",
     [](int *kind) { *kind = KIND_JR_XBUS; return jrxbus::detect() ? 250000UL : 0UL; }},
    {"Hitec HSR        19200 bps",
     [](int *kind) { *kind = KIND_HITEC; return hitec::detect() ? 19200UL : 0UL; }},
    {"Feetech STS/SCS  other baud rates",
     [](int *kind) {
       uint8_t series;
       unsigned long baud = feetech::detect(false, &series);
       *kind = series ? KIND_FEETECH_SCS : KIND_FEETECH_STS;
       return baud;
     }},
    {"Dynamixel        other baud rates",
     [](int *kind) { *kind = KIND_DYNAMIXEL; return dynamixel::detect(false); }},
    {"Futaba TTL       other baud rates",
     [](int *kind) { *kind = KIND_FUTABA; return futaba::detect(false); }},
};
#define DETECT_STEP_COUNT (sizeof(DETECT_STEPS) / sizeof(DETECT_STEPS[0]))

// 再起動後の動作（ソフトウェア再起動をまたいで残る）
enum BootAction : uint32_t
{
  BOOT_NORMAL = 0,     // メニューを表示する
  BOOT_START_SAVED = 1, // メニューを出さずに保存した種類で始める
  BOOT_DETECT = 2,      // すぐに自動判別する
};
RTC_NOINIT_ATTR static uint32_t bootMagic;
RTC_NOINIT_ATTR static uint32_t bootAction;
RTC_NOINIT_ATTR static uint32_t bootBaud; // BOOT_START_SAVED で使う通信速度（0: 種類の初期値）

static int activeKind = -1; // 動作中のサーボ種類

// NVSに保存した種類（なければ-1）
static int loadServoKind()
{
  Preferences prefs;
  prefs.begin("servosarego", true);
  int kind = prefs.getUChar(NVS_KEY_KIND, 0xFF);
  prefs.end();
  return kind < (int)SERVO_KIND_COUNT ? kind : -1;
}

static void saveServoKind(int kind)
{
  Preferences prefs;
  prefs.begin("servosarego", false);
  prefs.putUChar(NVS_KEY_KIND, (uint8_t)kind);
  prefs.end();
}

// 再起動後の動作を残して再起動する
static void restartWith(BootAction action, unsigned long baud)
{
  Serial.println("Restarting...");
  Serial.flush();
  bootMagic = BOOT_MAGIC;
  bootAction = action;
  bootBaud = baud;
  ESP.restart();
}

// 種類ごとの調べ方で変わったUARTとピンの設定を元に戻す
static void releaseServoBus()
{
  Serial1.end();
  GPIO.func_out_sel_cfg[TX1].oen_sel = 0; // TXの出力有効をUART側に戻す（エコーあり回路の設定を解除）
  gpio_pulldown_dis((gpio_num_t)RX1);    // Hitec の反転信号用のプルダウンを解除
}

// PWMサーボを試すか聞き、試すなら往復させて動いたかを確かめてもらう。
// PWMを先に受けるとPWMモードに固定されるシリアルサーボ（XBUSなど）があるため、聞いてから出す
static bool tryPwmServo()
{
  Serial.println("Try PWM servo? A 50 Hz PWM sweep is sent on the signal line.");
  Serial.println("(Some serial servos, e.g. XBUS, stay in PWM mode until power-cycled.)");
  Serial.println("[y] Try / Enter: skip");
  while (!Serial.available())
    delay(10);
  String input = Serial.readString();
  input.trim();
  if (input != "y")
    return false;

  bool moved = pwm::detectByMotion();
  Serial.println(moved ? "PWM servo confirmed." : "PWM servo not confirmed.");
  return moved;
}

// つながっているサーボの種類と通信速度を自動判別する。見つからなければ-1
static int detectServoKind(unsigned long *baud)
{
  Serial.println("============================================");
  Serial.println("Detecting servo type (read commands only, servos do not move)...");
  Serial.println("Scan range: ID0-31 (Kondo), ID0-20 (Feetech/Dynamixel), ID1-20 (Futaba/XBUS)");

  int found = -1;
  for (size_t i = 0; i < DETECT_STEP_COUNT && found < 0; i++)
  {
    Serial.print("  ");
    Serial.print(DETECT_STEPS[i].label);
    Serial.print(" ... ");

    int kind;
    unsigned long foundBaud = DETECT_STEPS[i].probe(&kind);
    releaseServoBus();

    if (foundBaud > 0)
    {
      Serial.println("found");
      found = kind;
      *baud = foundBaud;
    }
    else
    {
      Serial.println("-");
    }
  }

  if (found < 0)
  {
    Serial.println("No serial servo detected. (Hitec reads need only ONE servo on the bus.)");
    if (tryPwmServo())
    {
      *baud = 0;
      return KIND_PWM;
    }
    Serial.println("No servo detected. Check wiring/power, or select the type manually.");
    return -1;
  }

  Serial.print("Detected: ");
  Serial.print(SERVO_KINDS[found].name);
  Serial.print(" at ");
  Serial.print(*baud);
  Serial.println(" bps");
  return found;
}

// 種類の一覧を表示して番号を入力させる（[0] は KIND_DETECT を返す）。
// Enterだけ・時間切れ（timeoutMs > 0 のとき）は defaultKind を返す。defaultKind が-1なら選ぶまで待つ
static int askServoKind(int defaultKind, unsigned long timeoutMs)
{
  while (true)
  {
    Serial.println("============================================");
    Serial.println("ServosAreGo - Select servo type:");
    Serial.println("  [0] Detect servo type");
    for (size_t i = 0; i < SERVO_KIND_COUNT; i++)
    {
      Serial.print("  [");
      Serial.print(i + 1);
      Serial.print("] ");
      Serial.print(SERVO_KINDS[i].name);
      if ((int)i == defaultKind)
        Serial.print("  <- current");
      Serial.println();
    }
    if (defaultKind >= 0)
    {
      Serial.print("Enter number (Enter: keep current");
      if (timeoutMs > 0)
      {
        Serial.print(", auto start in ");
        Serial.print(timeoutMs / 1000);
        Serial.print(" s");
      }
      Serial.println("):");
    }
    else
    {
      Serial.println("Enter number:");
    }

    // 入力待ち
    unsigned long startTime = millis();
    while (!Serial.available())
    {
      if (defaultKind >= 0 && timeoutMs > 0 && millis() - startTime >= timeoutMs)
        return defaultKind;
      delay(10);
    }

    String input = Serial.readString();
    input.trim();
    if (input.length() == 0 && defaultKind >= 0)
      return defaultKind;
    if (input == "0")
      return KIND_DETECT;

    int number = input.toInt();
    if (number >= 1 && number <= (int)SERVO_KIND_COUNT)
      return number - 1;

    Serial.println("Error: Invalid selection");
    timeoutMs = 0; // 一度入力があったら自動起動しない
  }
}

void selectServoTypeAndRestart()
{
  int kind = askServoKind(activeKind, 0);

  // 動作中の種類がバスを使っているため、自動判別は再起動してから行う
  if (kind == KIND_DETECT)
  {
    restartWith(BOOT_DETECT, 0);
    return;
  }

  if (kind == activeKind)
  {
    Serial.print("Servo type unchanged: ");
    Serial.println(SERVO_KINDS[activeKind].name);
    return;
  }

  saveServoKind(kind);
  Serial.print("Servo type changed to: ");
  Serial.println(SERVO_KINDS[kind].name);
  restartWith(BOOT_START_SAVED, 0); // 再起動後はメニューを出さずにすぐ始める
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  // 再起動前に残した動作（電源投入時などは無効）
  BootAction action = BOOT_NORMAL;
  unsigned long baud = 0;
  if (esp_reset_reason() == ESP_RST_SW && bootMagic == BOOT_MAGIC)
  {
    action = (BootAction)bootAction;
    baud = bootBaud;
  }
  bootMagic = 0;

  int savedKind = loadServoKind();
  int kind;
  if (action == BOOT_START_SAVED && savedKind >= 0)
    kind = savedKind;
  else if (action == BOOT_DETECT)
    kind = KIND_DETECT;
  else
    kind = askServoKind(savedKind, SELECT_TIMEOUT);

  // 自動判別（見つからなければ手動で選ぶ）
  while (kind == KIND_DETECT)
  {
    kind = detectServoKind(&baud);
    if (kind < 0)
      kind = askServoKind(savedKind, 0);
  }

  activeKind = kind;
  if (activeKind != savedKind)
    saveServoKind(activeKind);

  Serial.print("Servo type: ");
  Serial.println(SERVO_KINDS[activeKind].name);
  if (baud > 0 && SERVO_KINDS[activeKind].presetBaud)
    SERVO_KINDS[activeKind].presetBaud(baud);
  SERVO_KINDS[activeKind].setup();
}

void loop()
{
  SERVO_KINDS[activeKind].loop();
}
