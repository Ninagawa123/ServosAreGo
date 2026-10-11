// ServosAreGo: 各社シリアルサーボの設定ツール（Feetech / Dynamixel / Futaba / JR XBUS / Hitec / Kondo / PWM）
//
// 起動するとサーボ種類の選択メニューを表示し、選んだ種類の Servo Setter を動かす。
// [0] を選ぶと、つながっているサーボの種類と通信速度を自動判別する。
// シリアルサーボが見つからなければ、確認したうえでPWMで動かし、動いたかを人に確かめてもらう。
// 選んだ種類はNVSに保存し、次回は入力がなければ同じ種類で起動する。
// 動作中は [h] でサーボ種類を選び直せる（選び直すと再起動する）。
// 配線（① 半二重回路あり / ② 半二重回路なし: RXとTXを両方信号線へ）は起動メニューの [w] で切り替え、NVSに保存する。
// ブラウザ版（web/ServosAreGo.html）からは "@" で始まる機械向けコマンドで操作し、結果は "@{JSON}" の1行で返す。

#include <Arduino.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include "servo_select.h"

#define FW_VERSION "0.1.0"          // ブラウザ版が書き込み済みの版を確かめるのに使う
#define SELECT_TIMEOUT 5000          // 前回の種類で自動起動するまでの待ち(ms)
#define BOOT_MAGIC 0x53415247UL      // 再起動後の動作（bootAction）が有効である目印
#define NVS_KEY_KIND "type"         // 種類の保存キー（並びを変えたら名前も変え、古い番号を使わない）
#define NVS_KEY_WIRING "wiring"     // 配線の保存キー（0: 半二重回路あり, 1: なし）
#define KIND_DETECT -2               // askServoKind() の戻り値: [0] 自動判別
#define KIND_NONE -3                 // handleMenuApi() の戻り値: 種類は選ばれていない
#define API_SCAN_MAX 64              // @scan で返すサーボの最大数
#define API_DEFAULT_SCAN_MAX_ID 20   // @scan の既定の最大ID

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

// ブラウザ用APIの関数表（各 namespace の api* 関数）
struct ServoApiTable
{
  void (*getInfo)(ApiInfo *info);
  int (*scan)(int maxId, ApiServo *found, int maxCount);
  bool (*setTarget)(int id);
  bool (*setSync)(const uint8_t *ids, uint8_t count);
  bool (*move)(float deg);
  bool (*center)();
  bool (*torque)(bool on);
  bool (*txTest)();
  bool (*readPosition)(int id, float *deg);
  bool (*setId)(int oldId, int newId);
  bool (*setBaud)(int index, bool servoToo);
};
#define SERVO_API_TABLE(ns)                                                                         \
  {ns::apiGetInfo, ns::apiScan, ns::apiSetTarget, ns::apiSetSync, ns::apiMove, ns::apiCenter,       \
   ns::apiTorque, ns::apiTxTest, ns::apiReadPosition, ns::apiSetId, ns::apiSetBaud}

const ServoApiTable API_PWM = SERVO_API_TABLE(pwm);
const ServoApiTable API_FEETECH = SERVO_API_TABLE(feetech);
const ServoApiTable API_DYNAMIXEL = SERVO_API_TABLE(dynamixel);
const ServoApiTable API_FUTABA = SERVO_API_TABLE(futaba);
const ServoApiTable API_JR_XBUS = SERVO_API_TABLE(jrxbus);
const ServoApiTable API_HITEC = SERVO_API_TABLE(hitec);
const ServoApiTable API_KONDO = SERVO_API_TABLE(kondo);

// サーボ種類の一覧（[1] がPWM。Feetech は STS と SCS で位置の範囲とバイト順が違うため分ける）
struct ServoKind
{
  const char *name;
  void (*setup)();
  void (*loop)();
  void (*presetBaud)(unsigned long baud); // 通信速度が固定の種類はNULL
  const ServoApiTable *api;               // ブラウザ用API
};

const ServoKind SERVO_KINDS[] = {
    {"PWM servo (50 Hz, no reply)", pwm::setup, pwm::loop, NULL, &API_PWM},
    {"Feetech STS (STS/SMS series)", [] { feetech::setup(0); }, feetech::loop, feetech::presetBaud, &API_FEETECH},
    {"Feetech SCS (SCS series)", [] { feetech::setup(1); }, feetech::loop, feetech::presetBaud, &API_FEETECH},
    {"Dynamixel (Protocol 2.0)", dynamixel::setup, dynamixel::loop, dynamixel::presetBaud, &API_DYNAMIXEL},
    {"Futaba TTL (RS30x)", futaba::setup, futaba::loop, futaba::presetBaud, &API_FUTABA},
    {"JR XBUS", jrxbus::setup, jrxbus::loop, NULL, &API_JR_XBUS},
    {"Hitec HSR (HMI)", hitec::setup, hitec::loop, NULL, &API_HITEC},
    {"Kondo KRS (ICS3.5/3.6)", kondo::setup, kondo::loop, kondo::presetBaud, &API_KONDO},
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
bool busDirectWiring = false; // 配線 true: ② 半二重回路なし（servo_bus.h が参照する）
static bool startWiring;      // 起動時の配線（変わったら再起動して反映する）

static const char *wiringText(bool direct)
{
  return direct ? "(2) No half-duplex circuit (RX+TX both on signal line)" : "(1) Half-duplex circuit (Meridian Board etc.)";
}

static bool loadWiring()
{
  Preferences prefs;
  prefs.begin("servosarego", true);
  bool direct = prefs.getUChar(NVS_KEY_WIRING, 0) == 1;
  prefs.end();
  return direct;
}

static void saveWiring(bool direct)
{
  Preferences prefs;
  prefs.begin("servosarego", false);
  prefs.putUChar(NVS_KEY_WIRING, direct ? 1 : 0);
  prefs.end();
}

// 配線を変えて保存する（反映は次の setup から。動作中なら再起動が必要）
static void setWiring(bool direct)
{
  busDirectWiring = direct;
  saveWiring(direct);
  Serial.print("Wiring: ");
  Serial.println(wiringText(direct));
}

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

// ===== ブラウザ用: JSON出力 =====

// JSON文字列（" と \ をエスケープ）
static void printJsonString(const char *text)
{
  Serial.print('"');
  for (const char *p = text; *p; p++)
  {
    if (*p == '"' || *p == '\\')
      Serial.print('\\');
    if ((uint8_t)*p >= 0x20)
      Serial.print(*p);
  }
  Serial.print('"');
}

// イベント（ブラウザに状態の変化を知らせる）: @{"ev":"<name>"<extra>}
static void printEvent(const char *name, const String &extra = "")
{
  Serial.print("@{\"ev\":\"");
  Serial.print(name);
  Serial.print('"');
  Serial.print(extra);
  Serial.println("}");
}

// 動作中の種類の状態: "info":{...}
static void printInfoJson()
{
  ApiInfo info;
  SERVO_KINDS[activeKind].api->getInfo(&info);
  Serial.print("\"info\":{\"idMin\":");
  Serial.print(info.idMin);
  Serial.print(",\"idMax\":");
  Serial.print(info.idMax);
  Serial.print(",\"angleMin\":");
  Serial.print(info.angleMin, 1);
  Serial.print(",\"angleMax\":");
  Serial.print(info.angleMax, 1);
  Serial.print(",\"bauds\":[");
  for (uint8_t i = 0; info.bauds && i < info.baudCount; i++)
  {
    if (i > 0)
      Serial.print(',');
    Serial.print(info.bauds[i]);
  }
  Serial.print("],\"baudIndex\":");
  Serial.print(info.baudIndex);
  Serial.print(",\"baud\":");
  Serial.print(info.baud);
  Serial.print(",\"target\":");
  Serial.print(info.target);
  Serial.print(",\"sync\":[");
  for (uint8_t i = 0; info.syncIds && i < info.syncCount; i++)
  {
    if (i > 0)
      Serial.print(',');
    Serial.print(info.syncIds[i]);
  }
  Serial.print("],\"oneServoForId\":");
  Serial.print(info.oneServoForId ? "true" : "false");
  Serial.print('}');
}

// 版数・種類の一覧・今の種類: "fw":...,"kinds":[...],"stage":...,"kind":k[,"info":{...}]
static void printHelloFields(const char *stage, int kind)
{
  Serial.print(",\"fw\":\"ServosAreGo\",\"ver\":\"" FW_VERSION "\",\"kinds\":[");
  for (size_t i = 0; i < SERVO_KIND_COUNT; i++)
  {
    if (i > 0)
      Serial.print(',');
    printJsonString(SERVO_KINDS[i].name);
  }
  Serial.print("],\"stage\":\"");
  Serial.print(stage);
  Serial.print("\",\"kind\":");
  Serial.print(kind);
  Serial.print(",\"wiring\":");
  Serial.print(busDirectWiring ? 1 : 0);
  if (activeKind >= 0)
  {
    Serial.print(',');
    printInfoJson();
  }
}

// 返信の始まり: @{"re":"<cmd>","ok":<ok>
static void beginReply(const String &cmd, bool ok)
{
  Serial.print("@{\"re\":");
  printJsonString(cmd.c_str());
  Serial.print(",\"ok\":");
  Serial.print(ok ? "true" : "false");
}

// 返信の終わり（動作中なら今の状態を付ける）
static void endReply()
{
  if (activeKind >= 0)
  {
    Serial.print(',');
    printInfoJson();
  }
  Serial.println('}');
}

static void replyResult(const String &cmd, bool ok)
{
  beginReply(cmd, ok);
  endReply();
}

// 再起動後の動作を残して再起動する
static void restartWith(BootAction action, unsigned long baud)
{
  printEvent("restart");
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
  printEvent("askpwm");
  while (!Serial.available())
    delay(10);
  String input = Serial.readStringUntil('\n');
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
  printEvent("detecting", ",\"steps\":" + String(DETECT_STEP_COUNT));

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
    printEvent("detectstep", ",\"step\":" + String(i) + ",\"label\":\"" + DETECT_STEPS[i].label + "\",\"found\":" +
                                 String(foundBaud > 0 ? "true" : "false"));
  }

  if (found < 0)
  {
    Serial.println("No serial servo detected. (Hitec reads need only ONE servo on the bus.)");
    if (tryPwmServo())
    {
      *baud = 0;
      printEvent("detected", ",\"kind\":" + String(KIND_PWM) + ",\"baud\":0");
      return KIND_PWM;
    }
    Serial.println("No servo detected. Check wiring/power, or select the type manually.");
    printEvent("detected", ",\"kind\":-1");
    return -1;
  }

  Serial.print("Detected: ");
  Serial.print(SERVO_KINDS[found].name);
  Serial.print(" at ");
  Serial.print(*baud);
  Serial.println(" bps");
  printEvent("detected", ",\"kind\":" + String(found) + ",\"baud\":" + String(*baud));
  return found;
}

// 起動メニューでのブラウザのコマンド。選ばれた種類（KIND_DETECT 含む）か、続けて待つなら KIND_NONE を返す
static int handleMenuApi(const String &line, int defaultKind)
{
  int space = line.indexOf(' ');
  String cmd = line.substring(1, space < 0 ? line.length() : space);
  String arg = space < 0 ? "" : line.substring(space + 1);

  if (cmd == "hello")
  {
    beginReply(cmd, true);
    printHelloFields("menu", defaultKind);
    Serial.println('}');
    return KIND_NONE;
  }
  if (cmd == "select")
  {
    int kind = arg.toInt();
    bool ok = arg.length() > 0 && kind >= 0 && kind < (int)SERVO_KIND_COUNT;
    replyResult(cmd, ok);
    return ok ? kind : KIND_NONE;
  }
  if (cmd == "detect")
  {
    replyResult(cmd, true);
    return KIND_DETECT;
  }
  if (cmd == "wiring")
  {
    bool ok = arg == "0" || arg == "1";
    if (ok)
      setWiring(arg == "1");
    replyResult(cmd, ok);
    return KIND_NONE;
  }
  replyResult(cmd, false); // 種類を選ぶまでは他のコマンドは使えない
  return KIND_NONE;
}

// 種類の一覧を表示して番号を入力させる（[0] は KIND_DETECT を返す）。
// Enterだけ・時間切れ（timeoutMs > 0 のとき）は defaultKind を返す。defaultKind が-1なら選ぶまで待つ
static int askServoKind(int defaultKind, unsigned long timeoutMs)
{
  while (true)
  {
    Serial.println("============================================");
    Serial.println("ServosAreGo - Select servo type:");
    Serial.print("  Wiring: ");
    Serial.println(wiringText(busDirectWiring));
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
    Serial.print("  [w] Change wiring -> ");
    Serial.println(wiringText(!busDirectWiring));
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

    printEvent("menu", ",\"kind\":" + String(defaultKind) + ",\"timeout\":" + String(timeoutMs));

    // 入力待ち（ブラウザのコマンドは、メニューを出し直さずに続けて待つ）
    String input;
    do
    {
      unsigned long startTime = millis();
      while (!Serial.available())
      {
        if (defaultKind >= 0 && timeoutMs > 0 && millis() - startTime >= timeoutMs)
          return defaultKind;
        delay(10);
      }
      input = Serial.readStringUntil('\n');
      input.trim();
      if (input.startsWith("@"))
      {
        timeoutMs = 0; // ブラウザがつながったら自動起動しない
        int kind = handleMenuApi(input, defaultKind);
        if (kind != KIND_NONE)
          return kind;
      }
    } while (input.startsWith("@"));

    if (input.length() == 0 && defaultKind >= 0)
      return defaultKind;
    if (input == "0")
      return KIND_DETECT;
    if (input == "w")
    {
      setWiring(!busDirectWiring);
      timeoutMs = 0;
      continue;
    }

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

  if (kind == activeKind && busDirectWiring == startWiring)
  {
    Serial.print("Servo type unchanged: ");
    Serial.println(SERVO_KINDS[activeKind].name);
    return;
  }

  saveServoKind(kind);
  Serial.print("Servo type: ");
  Serial.println(SERVO_KINDS[kind].name);
  restartWith(BOOT_START_SAVED, 0); // 再起動後はメニューを出さずにすぐ始める（配線の変更もここで反映）
}

// ===== ブラウザ用: 動作中のコマンド =====
//
// @hello / @info                  版数と今の状態
// @select <kind>                  種類を選んで再起動（kind = メニューの番号 - 1）
// @detect                         再起動して自動判別
// @scan [maxId]                   今の通信速度で検索（ターゲットと一斉制御リストも設定）
// @target <id> / @sync <id,id,..> ターゲットID / 一斉制御リスト
// @move <deg> / @center           一斉制御リストを角度へ / 中央へ
// @torque <0|1>                   トルクOFF（脱力）/ ON（保持）
// @txtest                         送信テスト（[t]）
// @pos <id>                       現在位置(度)
// @setid <old> <new>              ID変更
// @baud <index> <0|1>             通信速度の変更（1: サーボも, 0: ESP32だけ）
// @wiring <0|1>                   配線（0: 半二重回路あり, 1: なし）。変わったら再起動
void handleApiCommand(const String &line)
{
  String text = line;
  text.trim();
  int space = text.indexOf(' ');
  String cmd = text.substring(1, space < 0 ? text.length() : space);
  String arg = space < 0 ? "" : text.substring(space + 1);
  arg.trim();
  int space2 = arg.indexOf(' ');
  String arg1 = space2 < 0 ? arg : arg.substring(0, space2);
  String arg2 = space2 < 0 ? "" : arg.substring(space2 + 1);
  const ServoApiTable *api = SERVO_KINDS[activeKind].api;

  if (cmd == "hello" || cmd == "info")
  {
    beginReply(cmd, true);
    printHelloFields("run", activeKind);
    Serial.println('}');
  }
  else if (cmd == "select")
  {
    int kind = arg.toInt();
    bool ok = arg.length() > 0 && kind >= 0 && kind < (int)SERVO_KIND_COUNT;
    replyResult(cmd, ok);
    if (ok && kind != activeKind)
    {
      saveServoKind(kind);
      restartWith(BOOT_START_SAVED, 0);
    }
  }
  else if (cmd == "detect")
  {
    replyResult(cmd, true);
    restartWith(BOOT_DETECT, 0);
  }
  else if (cmd == "wiring")
  {
    bool ok = arg == "0" || arg == "1";
    if (ok)
      setWiring(arg == "1");
    replyResult(cmd, ok);
    if (ok && busDirectWiring != startWiring)
      restartWith(BOOT_START_SAVED, 0);
  }
  else if (cmd == "scan")
  {
    static ApiServo found[API_SCAN_MAX];
    int maxId = arg.length() > 0 ? arg.toInt() : API_DEFAULT_SCAN_MAX_ID;
    int count = api->scan(maxId, found, API_SCAN_MAX);
    beginReply(cmd, count >= 0);
    Serial.print(",\"servos\":[");
    for (int i = 0; i < count; i++)
    {
      if (i > 0)
        Serial.print(',');
      Serial.print("{\"id\":");
      Serial.print(found[i].id);
      Serial.print(",\"model\":");
      printJsonString(found[i].model);
      Serial.print('}');
    }
    Serial.print(']');
    endReply();
  }
  else if (cmd == "target")
  {
    replyResult(cmd, arg.length() > 0 && api->setTarget(arg.toInt()));
  }
  else if (cmd == "sync")
  {
    uint8_t ids[256];
    uint8_t count = 0;
    int start = 0;
    while (start < (int)arg.length() && count < sizeof(ids))
    {
      int comma = arg.indexOf(',', start);
      if (comma < 0)
        comma = arg.length();
      ids[count++] = (uint8_t)arg.substring(start, comma).toInt();
      start = comma + 1;
    }
    replyResult(cmd, count > 0 && api->setSync(ids, count));
  }
  else if (cmd == "move")
  {
    replyResult(cmd, arg.length() > 0 && api->move(arg.toFloat()));
  }
  else if (cmd == "center")
  {
    replyResult(cmd, api->center());
  }
  else if (cmd == "torque")
  {
    replyResult(cmd, api->torque(arg.toInt() != 0));
  }
  else if (cmd == "txtest")
  {
    replyResult(cmd, api->txTest());
  }
  else if (cmd == "pos")
  {
    float deg = 0;
    bool ok = arg.length() > 0 && api->readPosition(arg.toInt(), &deg);
    beginReply(cmd, ok);
    Serial.print(",\"id\":");
    Serial.print(arg.toInt());
    if (ok)
    {
      Serial.print(",\"deg\":");
      Serial.print(deg, 1);
    }
    endReply();
  }
  else if (cmd == "setid")
  {
    replyResult(cmd, arg1.length() > 0 && arg2.length() > 0 && api->setId(arg1.toInt(), arg2.toInt()));
  }
  else if (cmd == "baud")
  {
    replyResult(cmd, arg1.length() > 0 && api->setBaud(arg1.toInt(), arg2.toInt() != 0));
  }
  else
  {
    replyResult(cmd, false);
  }
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

  busDirectWiring = startWiring = loadWiring();
  Serial.print("Wiring: ");
  Serial.println(wiringText(busDirectWiring));

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

  // ブラウザ用: 起動完了と今の状態
  Serial.print("@{\"ev\":\"ready\"");
  printHelloFields("run", activeKind);
  Serial.println('}');
}

void loop()
{
  SERVO_KINDS[activeKind].loop();
}
