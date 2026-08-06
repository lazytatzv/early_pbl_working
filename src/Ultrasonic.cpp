#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int trigPin, int echoPin) {
    _trigPin = trigPin;
    _echoPin = echoPin;
}

void Ultrasonic::begin() {
    pinMode(_trigPin, OUTPUT);
    digitalWrite(_trigPin, LOW);
    pinMode(_echoPin, INPUT);
}

float Ultrasonic::getDistanceCm() {
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    
    // タイムアウトを設定して無限ループを防ぐ
    long duration = pulseIn(_echoPin, HIGH, TIMEOUT_US);

    if (duration == 0 || duration >= MAX_VALID_US) {
        return -1.0f; // 範囲外またはエラー
    }

    return duration / US_TO_CM; // cmに変換
}

float Ultrasonic::getDistanceMeter() {
    float cm = getDistanceCm();
    if (cm < 0) return -1.0f;
    return cm / 100.0f;
}
