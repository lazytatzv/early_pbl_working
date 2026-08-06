#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class Ultrasonic {
public:
    Ultrasonic(int trigPin, int echoPin);
    void begin();

    // データ取得用Method
    float getDistanceCm();
    float getDistanceMeter();

    // 超音波センサ内部定数
    static constexpr long  TIMEOUT_US   = 40000; // pulseIn タイムアウト（応答待ち ~6.8m分）
    static constexpr long  MAX_VALID_US = 38000; // 有効計測レンジの上限
    static constexpr float US_TO_CM     = 58.0f; // マイクロ秒 → cm 変換係数

private:
    int _trigPin;
    int _echoPin;
};

#endif
