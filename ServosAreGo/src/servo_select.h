// サーボ種類の選択と各 Servo Setter の入口（main.cpp と各 setter_*.cpp で共有）
//
// 各 setter_*.cpp は元の各 Servo Setter の main.cpp を namespace で包んだもの。
// main.cpp が起動時に選ばれた種類の setup()/loop() を呼ぶ。

#pragma once

#include <Arduino.h>

// [h]コマンド: サーボ種類を選び直して再起動する（選び直さなければそのまま戻る）
void selectServoTypeAndRestart();

// 自動判別の応答待ち(ms): 返信replyBytesバイトの受信時間（1バイト10ビット）+ 余裕5ms
static inline unsigned long detectTimeout(unsigned long baud, uint16_t replyBytes)
{
  return 5 + (replyBytes * 10000UL) / baud;
}

// 各種類の detect() は読み取りコマンドだけでサーボを探す（サーボは動かない）。
// 通信速度を探す種類は、見つかった速度を返し（なければ0）、presetBaud() でその速度から始められる

namespace feetech
{
void setup(uint8_t series); // series: 0 = STS, 1 = SCS
void loop();
unsigned long detect(bool defaultBaud, uint8_t *series);
void presetBaud(unsigned long baud);
} // namespace feetech

namespace dynamixel
{
void setup();
void loop();
unsigned long detect(bool defaultBaud);
void presetBaud(unsigned long baud);
} // namespace dynamixel

namespace futaba
{
void setup();
void loop();
unsigned long detect(bool defaultBaud);
void presetBaud(unsigned long baud);
} // namespace futaba

namespace jrxbus
{
void setup();
void loop();
bool detect();
} // namespace jrxbus

namespace hitec
{
void setup();
void loop();
bool detect();
} // namespace hitec

namespace kondo
{
void setup();
void loop();
unsigned long detect(bool defaultBaud);
void presetBaud(unsigned long baud);
} // namespace kondo

namespace pwm
{
void setup();
void loop();
bool detectByMotion(); // 往復させて、動いたかを人に確認してもらう（返信がないため）
} // namespace pwm
