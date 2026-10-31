// サーボ種類の選択と各 Servo Setter の入口（main.cpp と各 setter_*.cpp で共有）
//
// 各 setter_*.cpp は元の各 Servo Setter の main.cpp を namespace で包んだもの。
// main.cpp が起動時に選ばれた種類の setup()/loop() を呼ぶ。

#pragma once

#include <Arduino.h>

// [h]コマンド: サーボ種類を選び直して再起動する（選び直さなければそのまま戻る）
void selectServoTypeAndRestart();

// 機械向けコマンド（"@" で始まる行）を処理して、結果を "@{JSON}" の1行で返す（各 loop から呼ぶ）
void handleApiCommand(const String &line);

// 自動判別の応答待ち(ms): 返信replyBytesバイトの受信時間（1バイト10ビット）+ 余裕5ms
static inline unsigned long detectTimeout(unsigned long baud, uint16_t replyBytes)
{
  return 5 + (replyBytes * 10000UL) / baud;
}

// ===== ブラウザ用API（機械向けコマンド。main.cpp の handleApiCommand() が使う） =====
//
// 各 setter_*.cpp は namespace 内に下の api* 関数を実装する（対話入力はしない）。
// 処理中の説明は今までどおり Serial に表示してよい（ブラウザではターミナルに出る）。
// 対応していない操作は false（apiScan は -1）を返す。

#define API_MODEL_LEN 24

// 検索で見つかったサーボ
struct ApiServo
{
  uint8_t id;
  char model[API_MODEL_LEN]; // 型番など（不明なら "Unknown"）
};

// 今の状態と、その種類でできること
struct ApiInfo
{
  int idMin, idMax;           // 有効なID（IDのない種類は -1, -1）
  float angleMin, angleMax;   // 角度の範囲(度)
  const unsigned long *bauds; // 通信速度の一覧（変更できない種類は NULL）
  uint8_t baudCount;
  int baudIndex;              // 今の通信速度のインデックス（変更できない種類は -1）
  unsigned long baud;         // 今の通信速度(bps)。PWM は 0
  int target;                 // ターゲットID（IDのない種類は -1）
  const uint8_t *syncIds;     // 一斉制御リスト（IDのない種類は NULL）
  uint8_t syncCount;
  bool oneServoForId;         // ID変更（Hitec は検索も）がIDなしで、1台だけつなぐ必要がある
};

// 各種類が実装する関数（宣言は下の各 namespace にまとめて入れる）
#define SERVO_API_DECLARATIONS                                                                      \
  void apiGetInfo(ApiInfo *info);                                                                   \
  int apiScan(int maxId, ApiServo *found, int maxCount); /* 見つかった数。ターゲットとリストも設定 */ \
  bool apiSetTarget(int id);                                                                        \
  bool apiSetSync(const uint8_t *ids, uint8_t count);                                               \
  bool apiMove(float deg);              /* 一斉制御リストを角度へ（必要ならトルクON） */            \
  bool apiCenter();                     /* 一斉制御リストを中央へ */                                \
  bool apiTorque(bool on);              /* ON: トルクON・保持, OFF: 脱力 */                         \
  bool apiTxTest();                     /* [t] と同じ */                                            \
  bool apiReadPosition(int id, float *deg);                                                         \
  bool apiSetId(int oldId, int newId);  /* IDなしの種類は oldId を無視。書き込み後に読み返して確認 */ \
  bool apiSetBaud(int index, bool servoToo); /* servoToo: サーボも変更, false: ESP32 だけ */

// 各種類の detect() は読み取りコマンドだけでサーボを探す（サーボは動かない）。
// 通信速度を探す種類は、見つかった速度を返し（なければ0）、presetBaud() でその速度から始められる

// これ以下の通信速度は、データの連続したHIGH（最大 スタート後の8ビット+ストップ = 9ビット）が
// 0.47ms以上になり、PWMのパルスに似る（9600bps: 0.94ms）。PWMを先に受けるとPWMモードに固定される
// サーボ（Kondo / XBUS / Hitec / Futaba）があるため、全種類の自動判別では最後に調べる
#define DETECT_PWM_LIKE_MAX_BAUD 19200UL

// detect() で調べる通信速度（どれも速い順に調べる）
enum DetectBauds
{
  DETECT_DEFAULT_BAUD,     // 工場出荷時の速度だけ
  DETECT_OTHER_FAST_BAUDS, // それ以外で、PWMに似ない速度（DETECT_PWM_LIKE_MAX_BAUD より速い）
  DETECT_PWM_LIKE_BAUDS,   // それ以外で、PWMに似る速度（DETECT_PWM_LIKE_MAX_BAUD 以下）
  DETECT_ALL_BAUDS,        // すべての速度（種類を指定した自動判別）
};

// 通信速度を調べないなら true（isDefault: 工場出荷時の速度か）
static inline bool detectSkipsBaud(DetectBauds bauds, bool isDefault, unsigned long baud)
{
  switch (bauds)
  {
  case DETECT_DEFAULT_BAUD:
    return !isDefault;
  case DETECT_OTHER_FAST_BAUDS:
    return isDefault || baud <= DETECT_PWM_LIKE_MAX_BAUD;
  case DETECT_PWM_LIKE_BAUDS:
    return isDefault || baud > DETECT_PWM_LIKE_MAX_BAUD;
  default:
    return false;
  }
}

namespace feetech
{
SERVO_API_DECLARATIONS
void setup(uint8_t series); // series: 0 = STS, 1 = SCS
void loop();
unsigned long detect(DetectBauds bauds, uint8_t *series);
void presetBaud(unsigned long baud);
} // namespace feetech

namespace dynamixel
{
SERVO_API_DECLARATIONS
void setup();
void loop();
unsigned long detect(DetectBauds bauds);
void presetBaud(unsigned long baud);
} // namespace dynamixel

namespace futaba
{
SERVO_API_DECLARATIONS
void setup();
void loop();
unsigned long detect(DetectBauds bauds);
void presetBaud(unsigned long baud);
} // namespace futaba

namespace jrxbus
{
SERVO_API_DECLARATIONS
void setup();
void loop();
bool detect();
} // namespace jrxbus

namespace hitec
{
SERVO_API_DECLARATIONS
void setup();
void loop();
bool detect();
} // namespace hitec

namespace kondo
{
SERVO_API_DECLARATIONS
void setup();
void loop();
unsigned long detect(DetectBauds bauds);
void presetBaud(unsigned long baud);
} // namespace kondo

namespace pwm
{
SERVO_API_DECLARATIONS
void setup();
void loop();
bool detectByMotion(); // 往復させて、動いたかを人に確認してもらう（返信がないため）
} // namespace pwm
