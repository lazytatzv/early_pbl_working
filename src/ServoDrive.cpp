#include "ServoDrive.h"

ServoDrive servoDrive;

// =============================================================================
// ServoDrive Implementation
// =============================================================================

ServoDrive::ServoDrive() : pwm(Adafruit_PWMServoDriver()) {
    // キャリブレーションはコンストラクタではなく begin() で初期化する。
    // (Config.h の constexpr が確実に解決された後に読み込むため)
    for (int i = 0; i < 16; i++) {
        _calibs[i] = {0, 1, 0.0f, 180.0f};
        _lastPulses[i] = -1;
    }
}

void ServoDrive::begin() {
    pwm.begin();
    pwm.setPWMFreq(ServoConfig::SERVO_FREQ);
    _initCalibrations();
}

void ServoDrive::_initCalibrations() {
    using namespace ServoConfig;

    // テーブル駆動によるキャリブレーション初期化
    // {チャンネル, トリム角(int), 反転フラグ, 最小角, 最大角}
    struct CalibEntry {
        int   ch;
        float trim;   // float のまま保持（int キャスト不要）
        bool  invert;
        float minAng;
        float maxAng;
    };

    const CalibEntry table[] = {
        {FR_HIP,  FR_HIP_TRIM,  INVERT_FR_HIP,  FR_HIP_MIN,  FR_HIP_MAX},
        {FR_KNEE, FR_KNEE_TRIM, INVERT_FR_KNEE, FR_KNEE_MIN, FR_KNEE_MAX},
        {BL_HIP,  BL_HIP_TRIM,  INVERT_BL_HIP,  BL_HIP_MIN,  BL_HIP_MAX},
        {BL_KNEE, BL_KNEE_TRIM, INVERT_BL_KNEE, BL_KNEE_MIN, BL_KNEE_MAX},
        {FL_HIP,  FL_HIP_TRIM,  INVERT_FL_HIP,  FL_HIP_MIN,  FL_HIP_MAX},
        {FL_KNEE, FL_KNEE_TRIM, INVERT_FL_KNEE, FL_KNEE_MIN, FL_KNEE_MAX},
        {BR_HIP,  BR_HIP_TRIM,  INVERT_BR_HIP,  BR_HIP_MIN,  BR_HIP_MAX},
        {BR_KNEE, BR_KNEE_TRIM, INVERT_BR_KNEE, BR_KNEE_MIN, BR_KNEE_MAX},
        {SENSOR,  0.0f,          false,           SENSOR_MIN,  SENSOR_MAX},
    };

    for (const auto& e : table) {
        _calibs[e.ch] = {e.trim, e.invert ? -1 : 1, e.minAng, e.maxAng};
        // offset は float のまま格納（精度ロスなし）
    }
}

void ServoDrive::moveServo(int ch, float angle) {
    if (ch < 0 || ch >= 16) return;

    // 1. 安全のための角度クランプ (Joint Limits)
    float target = angle;
    if (target < _calibs[ch].minAng) target = _calibs[ch].minAng;
    if (target > _calibs[ch].maxAng) target = _calibs[ch].maxAng;

    // 2. オフセット適用
    target += _calibs[ch].offset;

    // 3. 回転方向の反転適用
    if (_calibs[ch].direction == -1) {
        target = 180.0f - target;
    }

    // 4. パルス幅へのマッピング
    int pulse;
    if (ch == ServoConfig::SENSOR) {
        // S05NF STD (900us 〜 2100us) ➔ 12-bit PCA9685値で 185 〜 430
        pulse = map((int)(target * 10), 0, 1800, 185, 430);
    } else {
        pulse = map((int)(target * 10), 0, 1800, ServoConfig::PULSE_MIN, ServoConfig::PULSE_MAX);
    }
    
    // 値が変化した時のみI2Cパケットを送信してバスの輻輳を回避
    if (pulse != _lastPulses[ch]) {
        pwm.setPWM(ch, 0, pulse);
        _lastPulses[ch] = pulse;
    }
}
