#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/**
 * =============================================================================
 *   ROBOT CONFIGURATION (AUTO-GENERATED FROM robot_config.yaml - DO NOT EDIT DIRECTLY)
 * =============================================================================
 */

// ファームウェアバージョン (文字列連結で F() マクロと組み合わせ可能)
#define FIRMWARE_VERSION "1.4"

namespace HardwareConfig {
    const int     ULTRASONIC_TRIG = 10;
    const int     ULTRASONIC_ECHO = 11;
    const uint8_t PCA9685_ADDR    = 0x40;  // PCA9685 I2Cアドレス
}

namespace ServoConfig {
    const int SERVO_FREQ = 50;

    enum Channel {
        FR_HIP = 0, FR_KNEE = 1,
        BL_HIP = 6, BL_KNEE = 7,
        FL_HIP = 2, FL_KNEE = 3,
        BR_HIP = 4, BR_KNEE = 5,
        SENSOR = 8,
        COUNT  = 9
    };

    const char* const JOINT_NAMES[] = {
        "FR_HIP", "FR_KNEE",
        "BL_HIP", "BL_KNEE",
        "FL_HIP", "FL_KNEE",
        "BR_HIP", "BR_KNEE",
        "SENSOR"
    };

    const bool INVERT_FR_HIP  = false;
    const bool INVERT_FR_KNEE = true;
    const bool INVERT_BL_HIP  = false;
    const bool INVERT_BL_KNEE = true;
    const bool INVERT_FL_HIP  = true;
    const bool INVERT_FL_KNEE = true;
    const bool INVERT_BR_HIP  = true;
    const bool INVERT_BR_KNEE = true;

    const float FR_HIP_TRIM  = 0.0f;
    const float FR_KNEE_TRIM = 0.0f;
    const float BL_HIP_TRIM  = 0.0f;
    const float BL_KNEE_TRIM = 0.0f;
    const float FL_HIP_TRIM  = 0.0f;
    const float FL_KNEE_TRIM = 0.0f;
    const float BR_HIP_TRIM  = 0.0f;
    const float BR_KNEE_TRIM = 0.0f;

    const int   PULSE_MIN      = 102;
    const int   PULSE_MAX      = 491;
    const float NEUTRAL_ANGLE  = 90.0f;

    const float FR_HIP_MIN  = 15.0f;
    const float FR_HIP_MAX  = 165.0f;
    const float FR_KNEE_MIN = 5.0f;
    const float FR_KNEE_MAX = 175.0f;

    const float BL_HIP_MIN  = 15.0f;
    const float BL_HIP_MAX  = 165.0f;
    const float BL_KNEE_MIN = 5.0f;
    const float BL_KNEE_MAX = 175.0f;

    const float FL_HIP_MIN  = 15.0f;
    const float FL_HIP_MAX  = 165.0f;
    const float FL_KNEE_MIN = 5.0f;
    const float FL_KNEE_MAX = 175.0f;

    const float BR_HIP_MIN  = 15.0f;
    const float BR_HIP_MAX  = 165.0f;
    const float BR_KNEE_MIN = 5.0f;
    const float BR_KNEE_MAX = 175.0f;

    const float SENSOR_MIN  = 0.0f;
    const float SENSOR_MAX  = 180.0f;
}

#include "UrdfConfig.h"

namespace RobotConfig {
    const float STANCE_OFFSET_X = 48.5f;
    const float STANCE_OFFSET_Y = 48.5f;
}

namespace GaitConfig {
    const float DEFAULT_Z         = 115.0f;
    const float STANCE_PULLDOWN_Z = 3.5f;
    const float STEP_HEIGHT       = 70.0f;
    const float STEP_LENGTH       = 75.0f;
    const int   CYCLE_TIME        = 1700;

    const float BODY_SHIFT_X   = 14.0f;
    const float BODY_SHIFT_Y   = 14.0f;
    const float SWING_RATIO    = 0.24f;
    const float LERP_RATE      = 0.30f;
    const float HEADING_ANGLE  = 270.0f;

    const float STRIDE_X_DIV   = 5.0f;
    const float STRIDE_Y_DIV   = 5.0f;
    const float STRIDE_ROT_DIV = 15.0f;
    const int   LEG_COUNT      = 4;     // 脚数 (固定値)
}

namespace ControlConfig {
    const float MOVE_SPEED            = 6.0f;
    const float TURN_SPEED            = 3.0f;
    const int   SENSOR_CHECK_INTERVAL = 50;
    const int   LOG_INTERVAL          = 1000;
    const int   INPUT_TIMEOUT_MS      = 200;  // 入力なしで停止までのタイムアウト (ms)
    const float OBSTACLE_DETECTION_DIST = 30.0f;  // 前方障害物検知距離 (cm)
    const float OBSTACLE_CLEARANCE_DIST = 30.0f;  // 側方・後方障害物クリア判定距離 (cm)
    const int   AUTO_FORWARD_FINAL_MS   = 11000; // 回避後の直進時間 (ms)
    const float ROUGH_STEP_HEIGHT       = 90.0f;  // 不整地モード時の足上げ高さ (mm)
    const int   ROUGH_CYCLE_TIME        = 2500;      // 不整地モード時の歩行周期時間 (ms)
    const bool  SENSOR_SERVO_ENABLED    = false; // 首振りサーボ有効化フラグ
    const int   AUTO_FORWARD_SIDE_MS    = 15000;   // 横移動（前進）する時間 (ms)
}

namespace ImuPidConfig {
    const uint8_t BNO055_ADDR    = 0x28;  // AD0=LOW: 0x28, AD0=HIGH: 0x29
    const bool    ENABLED        = true;
    const float   KP             = 0.4000f;
    const float   KI             = 0.0100f;
    const float   KD             = 0.0600f;
    const float   MAX_CORRECTION = 3.00f;
    const float   TURN_DEAD      = 0.10f;
}

#endif
