#include <Arduino.h>
#include "servo_select.h"

namespace pwm
{

// PWMサーボ（50Hz）: 信号線にパルスを出すだけで、サーボからの返信はない
// 半二重回路のENを送信方向に固定し、TXピンからLEDCでパルスを出す

typedef unsigned char u8;
typedef unsigned short u16;

// ピン設定
#define PIN_EN 33 // 半二重通信の送受信切替（HIGH: 送信, LOW: 受信）。PWMでは常に送信

// PWM設定
#define PWM_CHANNEL 0       // LEDCチャンネル
#define PWM_FREQUENCY 50    // 周波数(Hz) = 周期20ms
#define PWM_RESOLUTION 16   // デューティの分解能(bit)
#define PWM_PERIOD_US 20000 // 周期(us)

// パルス幅（一般的なサーボ: 1500usが中央、10us/度）
#define PULSE_CENTER 1500
#define PULSE_MIN 500  // 約-100度
#define PULSE_MAX 2500 // 約+100度
#define US_PER_DEGREE 10.0

// 送信テスト・判別用の往復
#define TEST_ANGLE 30.0
#define DETECT_TIMEOUT 10000 // 動きを確認する時間(ms)

u16 currentPulse = PULSE_CENTER; // 最後に出したパルス幅(us)
bool pulseOn = false;            // パルスを出しているか

String readInput()
{
  while (!Serial.available())
    delay(10);
  String input = Serial.readString();
  input.trim();
  return input;
}

u16 angleToPulse(float angle)
{
  float pulse = PULSE_CENTER + angle * US_PER_DEGREE;
  if (pulse < PULSE_MIN)
    pulse = PULSE_MIN;
  if (pulse > PULSE_MAX)
    pulse = PULSE_MAX;
  return (u16)pulse;
}

float pulseToAngle(u16 pulseUs)
{
  return ((float)pulseUs - PULSE_CENTER) / US_PER_DEGREE;
}

// パルス出力の開始（ENを送信方向にしてTXピンにLEDCをつなぐ）
void beginPwm()
{
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(TX1, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0); // パルスなし（線はLOW）
}

// パルス出力の終了（他の種類がTXピンを使えるように外す）
void endPwm()
{
  ledcWrite(PWM_CHANNEL, 0);
  ledcDetachPin(TX1);
}

// パルス幅を設定して出す
void setPulse(u16 pulseUs)
{
  currentPulse = pulseUs;
  pulseOn = true;
  ledcWrite(PWM_CHANNEL, (uint32_t)pulseUs * ((1UL << PWM_RESOLUTION) - 1) / PWM_PERIOD_US);
}

// パルスを止める（多くのサーボはパルスがないと脱力する）
void stopPulse()
{
  pulseOn = false;
  ledcWrite(PWM_CHANNEL, 0);
}

void printPulse(u16 pulseUs)
{
  Serial.print(pulseUs);
  Serial.print(" us (");
  Serial.print(pulseToAngle(pulseUs), 1);
  Serial.print(" deg)");
}

// [x]コマンド: 角度指令
void executeAngleCommand()
{
  Serial.print("Enter angle (");
  Serial.print(pulseToAngle(PULSE_MIN), 0);
  Serial.print(" to ");
  Serial.print(pulseToAngle(PULSE_MAX), 0);
  Serial.println(" degrees): ");

  String input = readInput();
  float angle = input.toFloat();
  if (input.length() == 0 || angle < pulseToAngle(PULSE_MIN) || angle > pulseToAngle(PULSE_MAX))
  {
    Serial.println("Error: Angle out of range");
    return;
  }

  setPulse(angleToPulse(angle));
  Serial.print("Pulse: ");
  printPulse(currentPulse);
  Serial.println();
}

// [w]コマンド: パルス幅を直接指定
void executeSetPulseCommand()
{
  Serial.print("Enter pulse width (");
  Serial.print(PULSE_MIN);
  Serial.print("-");
  Serial.print(PULSE_MAX);
  Serial.println(" us): ");

  String input = readInput();
  long pulse = input.toInt();
  if (input.length() == 0 || pulse < PULSE_MIN || pulse > PULSE_MAX)
  {
    Serial.println("Error: Pulse width out of range");
    return;
  }

  setPulse((u16)pulse);
  Serial.print("Pulse: ");
  printPulse(currentPulse);
  Serial.println();
}

// [c]コマンド: 中央へ
void executeCenterCommand()
{
  setPulse(PULSE_CENTER);
  Serial.print("Pulse: ");
  printPulse(currentPulse);
  Serial.println();
}

// [e]コマンド: パルス出力ON（最後のパルス幅で保持）
void executePulseOnCommand()
{
  setPulse(currentPulse);
  Serial.print("Pulse ON: ");
  printPulse(currentPulse);
  Serial.println();
}

// [d]コマンド: パルス出力OFF（脱力）
void executePulseOffCommand()
{
  stopPulse();
  Serial.println("Pulse OFF (most servos release when no pulse is received)");
}

// [t]コマンド: 送信テスト（+30, -30, +30, -30, 中央）
void executeTxTestCommand()
{
  Serial.println("TX test: +30, -30, +30, -30, center (PWM, no reply).");

  const float angles[] = {TEST_ANGLE, -TEST_ANGLE, TEST_ANGLE, -TEST_ANGLE, 0.0};
  for (u8 step = 0; step < sizeof(angles) / sizeof(angles[0]); step++)
  {
    setPulse(angleToPulse(angles[step]));
    Serial.print("  ");
    Serial.print(angles[step], 0);
    Serial.println("deg sent");
    delay(700);
  }

  Serial.println("TX test finished (pulse stays ON at center. Use [d] to release).");
  Serial.println("Not moved -> check power and wiring. Serial-mode servos ignore PWM.");
}

void disp_menu()
{
  Serial.println("============================================");
  Serial.print("Pulse: ");
  printPulse(currentPulse);
  Serial.print(pulseOn ? " ON" : " OFF");
  Serial.print(", Range: ");
  Serial.print(PULSE_MIN);
  Serial.print("-");
  Serial.print(PULSE_MAX);
  Serial.print(" us, ");
  Serial.print(US_PER_DEGREE, 0);
  Serial.print(" us/deg, ");
  Serial.print(PWM_FREQUENCY);
  Serial.println(" Hz, Protocol: PWM");
  Serial.println("Menu: [x] Send angle    [c] Center        [t] TX test (sweep)");
  Serial.println("      [e] Pulse ON      [d] Pulse OFF     [w] Set pulse width (us)");
  Serial.println("      [h] Change servo type (restart)");
  Serial.println("Note: PWM servos do not reply. Check by motion.");
  Serial.println("Input Command:");
}

void setup()
{
  Serial.begin(115200);
  beginPwm(); // 起動時はパルスを出さない（サーボは動かない）

  delay(1000);

  Serial.println("=====================================");
  Serial.println("PWM Servo Setter");
  Serial.println("50 Hz PWM on TX pin (no reply)");
  Serial.println("=====================================");
  Serial.println("Pulse is OFF. Use [c] / [x] / [t] to move the servo.");

  disp_menu();
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readString();
    command.trim();

    if (command == "x")
      executeAngleCommand();
    else if (command == "c")
      executeCenterCommand();
    else if (command == "t")
      executeTxTestCommand();
    else if (command == "e")
      executePulseOnCommand();
    else if (command == "d")
      executePulseOffCommand();
    else if (command == "w")
      executeSetPulseCommand();
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

// PWMサーボは返信しないため、往復させて動いたかを人に確認してもらう。
// Enterだけ: 動いた（true）, それ以外の入力・時間切れ: 動かなかった（false）。終了時はパルスを止めて外す
bool detectByMotion()
{
  beginPwm();
  Serial.print("Sweeping +/-");
  Serial.print(TEST_ANGLE, 0);
  Serial.println(" deg by PWM...");
  Serial.print("If the servo moves, press Enter. Otherwise type n + Enter (auto stop in ");
  Serial.print(DETECT_TIMEOUT / 1000);
  Serial.println(" s).");

  bool moved = false;
  unsigned long startTime = millis();
  while (millis() - startTime < DETECT_TIMEOUT)
  {
    if (Serial.available())
    {
      String input = Serial.readString();
      input.trim();
      moved = (input.length() == 0);
      break;
    }
    float elapsed = (millis() - startTime) / 1000.0;
    setPulse(angleToPulse(TEST_ANGLE * sin(2.0 * PI * elapsed)));
    delay(20);
  }

  stopPulse();
  currentPulse = PULSE_CENTER;
  endPwm();
  return moved;
}

} // namespace pwm
