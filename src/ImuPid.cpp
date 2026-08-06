#include "ImuPid.h"

ImuPid imuPid;

// =============================================================================
// ImuPid Implementation
// =============================================================================

ImuPid::ImuPid()
    : _bno(55, ImuPidConfig::BNO055_ADDR),
      _targetHeading(0.0f),
      _integral(0.0f),
      _prevError(0.0f),
      _ready(false) {}

bool ImuPid::begin() {
    if (!_bno.begin()) {
        Serial.println(F("[WARN] BNO055 not found. Heading PID disabled."));
        _ready = false;
        return false;
    }

    // 外部水晶発振子を使用（精度向上）
    _bno.setExtCrystalUse(true);
    delay(100);

    captureTarget();
    _ready = true;

    Serial.println(F("[Info] BNO055 ready. Heading PID enabled."));
    return true;
}

void ImuPid::captureTarget() {
    _targetHeading = getHeadingDeg();
    _integral      = 0.0f;
    _prevError     = 0.0f;
}

float ImuPid::compute(float dt) {
    if (!_ready || dt <= 0.0f) return 0.0f;

    float current = getHeadingDeg();
    float error   = _normalizeError(current - _targetHeading);

    // ── 積分項 (anti-windup クランプ) ──────────────────────────────────
    _integral += error * dt;
    // KI=0 のとき ゼロ除算を避ける
    if (ImuPidConfig::KI > 0.0f) {
        float maxIntegral = ImuPidConfig::MAX_CORRECTION / ImuPidConfig::KI;
        if (_integral >  maxIntegral) _integral =  maxIntegral;
        if (_integral < -maxIntegral) _integral = -maxIntegral;
    }

    // ── 微分項 ────────────────────────────────────────────────────────
    float derivative = (error - _prevError) / dt;
    _prevError = error;

    // ── PID 出力 ──────────────────────────────────────────────────────
    float output = ImuPidConfig::KP * error
                 + ImuPidConfig::KI * _integral
                 + ImuPidConfig::KD * derivative;

    // 出力クランプ
    if (output >  ImuPidConfig::MAX_CORRECTION) output =  ImuPidConfig::MAX_CORRECTION;
    if (output < -ImuPidConfig::MAX_CORRECTION) output = -ImuPidConfig::MAX_CORRECTION;

    return output;
}

float ImuPid::getHeadingDeg() {
    sensors_event_t event;
    _bno.getEvent(&event);
    return event.orientation.x;  // BNO055 Euler X = Heading (0〜359.99°)
}

float ImuPid::_normalizeError(float deg) {
    // 偏差を -180〜+180° の範囲に折り畳む (fmodf で算術的に解決）
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}
