#include "Commander.h"
#include <Wire.h>
#include "Gait.h"
#include "ServoDrive.h"
#include "Ultrasonic.h"
#include "Config.h"

Commander commander;

// 距離センサのグローバルインスタンスをこちらに移動
Ultrasonic ultrasonic(HardwareConfig::ULTRASONIC_TRIG, HardwareConfig::ULTRASONIC_ECHO);

Commander::Commander() 
    : _lastCommandTime(0), _lastSensorCheck(0), _lastLogTime(0), _lastDistance(-1.0f),
      _ultrasonicTestMode(false), _imuTestMode(false), _autoDriveMode(false), _forwardDetectStopMode(false),
      _roughTerrainMode(false), _savedNormalStepHeight(0.0f), _savedNormalCycleTime(0),
      _autoState(AUTO_IDLE), _stateStartTime(0), _targetHeadingSave(0.0f) {}


void Commander::begin() {
    Wire.begin();
    servoDrive.begin();
    gait.begin();
    ultrasonic.begin();
    imuPid.begin();  // BNO055 初期化 (未接続でも続行可)

    Serial.println(F("[Info] Commander System Ready."));
    Serial.println(F("[Controls] WASD/JK:Move, r:Stand, z:ZeroShift, t:Diag, p:SensorServoTest, u:UltrasonicTest, i:ImuTest, o:AutoDrive, f:ForwardDetectStop, c:RoughTerrain"));
    Serial.println(F("[Adjust] +/-: Step Height, [/]: Step Length, </>: Cycle Time"));
}

void Commander::update() {
    processSerialInput();
    
    if (_ultrasonicTestMode) {
        runUltrasonicTest();
        return;
    }

    if (_imuTestMode) {
        runImuTest();
        return;
    }

    if (_autoDriveMode) {
        updateSensors();
        updateAutoDrive();
        printStatus();
        return;
    }

    if (_forwardDetectStopMode) {
        updateSensors();
        updateForwardDetectStop();
        printStatus();
        return;
    }

    checkInputTimeout();
    updateSensors();
    printStatus();
}

void Commander::processSerialInput() {
    if (Serial.available() <= 0) return;

    char cmd = Serial.read();
    
    // 改行コードはログを出さずに無視
    if (cmd == '\n' || cmd == '\r') return;

    using namespace ControlConfig;
    bool valid = true;

    switch (cmd) {
        case 'w': 
            Serial.println(F("[Cmd] Move Forward (W)"));
            gait.setVelocity( MOVE_SPEED,  0,  0); 
            break;
        case 's': 
            Serial.println(F("[Cmd] Move Backward (S)"));
            gait.setVelocity(-MOVE_SPEED,  0,  0); 
            break;
        case 'a': 
            Serial.println(F("[Cmd] Slide Left (A)"));
            gait.setVelocity( 0,  MOVE_SPEED,  0); 
            break;
        case 'd': 
            Serial.println(F("[Cmd] Slide Right (D)"));
            gait.setVelocity( 0, -MOVE_SPEED,  0); 
            break;
        case 'j': 
            Serial.println(F("[Cmd] Turn Left (J)"));
            gait.setVelocity( 0,  0,  TURN_SPEED); 
            break;
        case 'k': 
            Serial.println(F("[Cmd] Turn Right (K)"));
            gait.setVelocity( 0,  0, -TURN_SPEED); 
            break;
        case 'r': 
            Serial.println(F("[Cmd] Reset to Stand (R)"));
            gait.stand(); 
            break;
        case 'u':
        case 'U':
            _ultrasonicTestMode = !_ultrasonicTestMode;
            _imuTestMode = false;
            _autoDriveMode = false;
            _forwardDetectStopMode = false;
            if (_ultrasonicTestMode) {
                gait.stop();
                Serial.println(F("\n========================================"));
                Serial.println(F("   ULTRASONIC SENSOR TEST MODE (ON)"));
                Serial.println(F("   Press 'u' again to exit test mode."));
                Serial.println(F("========================================"));
            } else {
                Serial.println(F("\n[Info] Exiting Ultrasonic Test Mode. Standing up..."));
                gait.stand();
            }
            valid = false;
            break;
        case 'i':
        case 'I':
            _imuTestMode = !_imuTestMode;
            _ultrasonicTestMode = false;
            _autoDriveMode = false;
            _forwardDetectStopMode = false;
            if (_imuTestMode) {
                gait.stop();
                Serial.println(F("\n========================================"));
                Serial.println(F("   IMU (BNO055) SENSOR TEST MODE (ON)"));
                Serial.println(F("   Press 'i' again to exit test mode."));
                Serial.println(F("========================================"));
            } else {
                Serial.println(F("\n[Info] Exiting IMU Test Mode. Standing up..."));
                gait.stand();
            }
            valid = false;
            break;
        case 'o':
        case 'O':
            _autoDriveMode = !_autoDriveMode;
            _ultrasonicTestMode = false;
            _imuTestMode = false;
            _forwardDetectStopMode = false;
            if (_autoDriveMode) {
                Serial.println(F("\n========================================"));
                Serial.println(F("   AUTONOMOUS DRIVE MODE (ON)"));
                Serial.println(F("   Press 'o' again to stop auto drive."));
                Serial.println(F("========================================"));
                changeAutoState(AUTO_FORWARD_INIT);
                if (imuPid.isReady()) {
                    imuPid.captureTarget();
                    _targetHeadingSave = imuPid.getHeadingDeg(); // 自動走行スタート時の「絶対基準向き」を保存
                }
                if (ControlConfig::SENSOR_SERVO_ENABLED) {
                    servoDrive.moveServo(ServoConfig::SENSOR, 90.0f); // 正面向き
                }
            } else {
                Serial.println(F("\n[Info] Stopping Autonomous Drive. Standing up..."));
                changeAutoState(AUTO_IDLE);
                gait.stand();
                if (ControlConfig::SENSOR_SERVO_ENABLED) {
                    servoDrive.moveServo(ServoConfig::SENSOR, 90.0f);
                }
            }
            valid = false;
            break;
        case 'f':
        case 'F':
            _forwardDetectStopMode = !_forwardDetectStopMode;
            _ultrasonicTestMode = false;
            _imuTestMode = false;
            _autoDriveMode = false;
            if (_forwardDetectStopMode) {
                Serial.println(F("\n========================================"));
                Serial.println(F("   FORWARD DETECT STOP MODE (ON)"));
                Serial.println(F("   Press 'f' again to exit mode."));
                Serial.println(F("========================================"));
                if (ControlConfig::SENSOR_SERVO_ENABLED) {
                    servoDrive.moveServo(ServoConfig::SENSOR, 90.0f); // 正面向き
                }
                gait.setVelocity(ControlConfig::MOVE_SPEED, 0, 0);
            } else {
                Serial.println(F("\n[Info] Stopping Forward Detect Stop Mode. Standing up..."));
                gait.setVelocity(0, 0, 0);
                gait.stand();
            }
            valid = false;
            break;
        case 'c':
        case 'C':
            _roughTerrainMode = !_roughTerrainMode;
            if (_roughTerrainMode) {
                _savedNormalStepHeight = gait.getStepHeight();
                _savedNormalCycleTime = gait.getCycleTime();
                gait.setStepHeight(ControlConfig::ROUGH_STEP_HEIGHT);
                gait.setCycleTime(ControlConfig::ROUGH_CYCLE_TIME);
                Serial.println(F("\n========================================"));
                Serial.println(F("   ROUGH TERRAIN MODE (ON)"));
                Serial.print(F("   Step Height: ")); Serial.print(ControlConfig::ROUGH_STEP_HEIGHT); Serial.println(F(" mm"));
                Serial.print(F("   Cycle Time:  ")); Serial.print(ControlConfig::ROUGH_CYCLE_TIME); Serial.println(F(" ms"));
                Serial.println(F("========================================"));
            } else {
                gait.setStepHeight(_savedNormalStepHeight);
                gait.setCycleTime(_savedNormalCycleTime);
                Serial.println(F("\n========================================"));
                Serial.println(F("   ROUGH TERRAIN MODE (OFF)"));
                Serial.print(F("   Restored Step Height: ")); Serial.print(gait.getStepHeight()); Serial.println(F(" mm"));
                Serial.print(F("   Restored Cycle Time:  ")); Serial.print(gait.getCycleTime()); Serial.println(F(" ms"));
                Serial.println(F("========================================"));
            }
            valid = false;
            break;
        case 'z':
        case 'Z': {
            bool currentDisabled = gait.isShiftDisabled();
            gait.setShiftDisabled(!currentDisabled);
            Serial.print(F("[Cmd] Body Shift Mode: "));
            Serial.println(!currentDisabled ? F("OFF (Shift Disabled)") : F("ON (Shift Enabled)"));
            valid = false;
            break;
        }
        case 't': 
            Serial.println(F("[Cmd] Run Diagnostic (T)"));
            runDiagnostic(); 
            valid = false; 
            break;
        case 'p':
        case 'P': {
            Serial.println(F("[Cmd] Sweep SENSOR Servo (P)"));
            Serial.println(F(" -> Target: Channel 8. Moving to 90 (Center)..."));
            servoDrive.moveServo(ServoConfig::SENSOR, 90);
            delay(1000);
            Serial.println(F(" -> Moving to 180 (Left)..."));
            servoDrive.moveServo(ServoConfig::SENSOR, 180);
            delay(1000);
            Serial.println(F(" -> Moving to 0 (Right)..."));
            servoDrive.moveServo(ServoConfig::SENSOR, 0);
            delay(1000);
            Serial.println(F(" -> Returning to 90 (Center)..."));
            servoDrive.moveServo(ServoConfig::SENSOR, 90);
            delay(500);
            Serial.println(F(" -> SENSOR Servo Test Complete."));
            valid = false;
            break;
        }
        case '+':
        case '=':
            gait.adjustStepHeight(5.0f);
            Serial.print(F("[Cmd] Step Height ++: "));
            Serial.println(gait.getStepHeight());
            valid = false;
            break;
        case '-':
            gait.adjustStepHeight(-5.0f);
            Serial.print(F("[Cmd] Step Height --: "));
            Serial.println(gait.getStepHeight());
            valid = false;
            break;
        case ']':
            gait.adjustStepLength(5.0f);
            Serial.print(F("[Cmd] Step Length ++: "));
            Serial.println(gait.getStepLength());
            valid = false;
            break;
        case '[':
            gait.adjustStepLength(-5.0f);
            Serial.print(F("[Cmd] Step Length --: "));
            Serial.println(gait.getStepLength());
            valid = false;
            break;
        case '.':
        case '>':
            gait.adjustCycleTime(100);
            Serial.print(F("[Cmd] Cycle Time ++ (Slower): "));
            Serial.println(gait.getCycleTime());
            valid = false;
            break;
        case ',':
        case '<':
            gait.adjustCycleTime(-100);
            Serial.print(F("[Cmd] Cycle Time -- (Faster): "));
            Serial.println(gait.getCycleTime());
            valid = false;
            break;
        default:  
            Serial.print(F("[Warn] Unknown key: '"));
            Serial.print(cmd);
            Serial.println(F("'"));
            valid = false; 
            break;
    }

    if (valid) _lastCommandTime = millis();
}

void Commander::checkInputTimeout() {
    if (_lastCommandTime != 0 && (millis() - _lastCommandTime > ControlConfig::INPUT_TIMEOUT_MS)) {
        Serial.println(F("[Info] Input timeout. Stopping..."));
        gait.setVelocity(0, 0, 0);
        _lastCommandTime = 0; // タイムアウトのログ出力を1回のみにする
    }
}

void Commander::updateSensors() {
    if (millis() - _lastSensorCheck > ControlConfig::SENSOR_CHECK_INTERVAL) {
        _lastDistance = ultrasonic.getDistanceCm();  // 結果をキャッシュ
        _lastSensorCheck = millis();
    }
}

void Commander::printStatus() {
    if (millis() - _lastLogTime > ControlConfig::LOG_INTERVAL) {
        Serial.print(F("[Status] Dist: "));
        Serial.print(_lastDistance);
        Serial.print(F(" cm"));
        if (imuPid.isReady()) {
            Serial.print(F(" | Heading: "));
            Serial.print(imuPid.getHeadingDeg(), 1);
            Serial.print(F("deg (target: "));
            Serial.print(imuPid.getTargetDeg(), 1);
            Serial.print(F("deg)"));
        }
        if (_autoDriveMode) {
            Serial.print(F(" | State: "));
            Serial.print(getAutoStateName(_autoState));
        }
        Serial.println();
        _lastLogTime = millis();
    }
}

void Commander::runDiagnostic() {
    gait.stop();
    Serial.println(F("\n--- HARDWARE DIAGNOSTIC START ---"));
    
    // PCA9685 サーボドライバの検出 (I2Cアドレス: HardwareConfig::PCA9685_ADDR)
    Wire.beginTransmission(HardwareConfig::PCA9685_ADDR);
    if (Wire.endTransmission() == 0) {
        Serial.println(F("[OK] PCA9685 Servo Driver detected"));
    } else {
        Serial.println(F("[ERR] PCA9685 NOT found!"));
    }
    
    float d = ultrasonic.getDistanceCm();
    if (d > 0) {
        Serial.print(F("[OK] Ultrasonic Sensor: ")); Serial.print(d); Serial.println(F(" cm"));
    } else {
        Serial.println(F("[ERR] Ultrasonic Sensor timeout!"));
    }
    
    Serial.println(F("Testing all servos (Check physical direction)..."));
    for (int i = 0; i < ServoConfig::COUNT; i++) {
        Serial.print(F(" -> [")); Serial.print(ServoConfig::JOINT_NAMES[i]); Serial.println(F("]"));
        
        // 各関節の設計上の期待される動作方向を表示
        if (i == ServoConfig::SENSOR) {
            Serial.println(F("    [Expect] 70: Face Right | 110: Face Left"));
        } else if (i % 2 == 0) { // HIP (0, 2, 4, 6)
            Serial.println(F("    [Expect] 70: Swing Backward | 110: Swing Forward"));
        } else { // KNEE (1, 3, 5, 7)
            Serial.println(F("    [Expect] 70: Extend (Down/Straight) | 110: Bend (Up/Retract)"));
        }
        
        Serial.print(F("    Moving to 70... "));
        servoDrive.moveServo(i, 70); 
        delay(1000); // 動作を目視確認するためディレイを延長
        
        Serial.print(F("110... "));
        servoDrive.moveServo(i, 110); 
        delay(1000);
        
        Serial.println(F("Back to 90."));
        servoDrive.moveServo(i, 90); 
        delay(500);
    }
    
    Serial.println(F("--- DIAGNOSTIC COMPLETE ---\n"));
    gait.stand();
}

void Commander::runUltrasonicTest() {
    if (millis() - _lastSensorCheck > 100) { // 100ms周期で距離を出力
        float d = ultrasonic.getDistanceCm();
        Serial.print(F("[Ultrasonic Test] Distance: "));
        if (d > 0) {
            Serial.print(d, 1);
            Serial.print(F(" cm"));
            if (d <= ControlConfig::OBSTACLE_DETECTION_DIST) {
                Serial.print(F("  <-- [DETECTED <= "));
                Serial.print(ControlConfig::OBSTACLE_DETECTION_DIST, 1);
                Serial.print(F("cm]"));
            }
        } else {
            Serial.print(F("Out of range / Timeout"));
        }
        Serial.println();
        _lastSensorCheck = millis();
    }
}

void Commander::runImuTest() {
    if (millis() - _lastSensorCheck > 20) {
        Serial.print(F("[IMU Test] Status: "));
        if (imuPid.isReady()) {
            float heading = imuPid.getHeadingDeg();
            Serial.print(F("READY | Current Heading: "));
            Serial.print(heading, 1);
            Serial.print(F(" deg | Target: "));
            Serial.print(imuPid.getTargetDeg(), 1);
            Serial.print(F(" deg"));
            
            uint8_t sys, gyro, accel, mag;
            imuPid.getCalibration(&sys, &gyro, &accel, &mag);
            Serial.print(F(" | Calib S:"));
            Serial.print(sys);
            Serial.print(F(" G:"));
            Serial.print(gyro);
            Serial.print(F(" A:"));
            Serial.print(accel);
            Serial.print(F(" M:"));
            Serial.print(mag);
        } else {
            Serial.print(F("NOT FOUND / DISABLED"));
        }
        Serial.println();
        _lastSensorCheck = millis();
    }
}

void Commander::updateAutoDrive() {
    using namespace ControlConfig;

    switch (_autoState) {
        case AUTO_FORWARD_INIT: {
            // 正面向きで前進
            gait.setVelocity(MOVE_SPEED, 0, 0);

            // 前方障害物を検知したら右90度旋回へ移行
            if (_lastDistance > 0 && _lastDistance <= OBSTACLE_DETECTION_DIST) {
                Serial.print(F("[Auto] Obstacle detected within "));
                Serial.print(OBSTACLE_DETECTION_DIST, 1);
                Serial.println(F("cm! Starting Right Turn (90 deg)..."));

                // 右旋回開始: 速度を即座にセット
                gait.setVelocity(0, 0, -TURN_SPEED);

                // PIDターゲットを右折目標角度に設定
                if (imuPid.isReady()) {
                    imuPid.setTargetDeg(fmodf(_targetHeadingSave + 90.0f + 360.0f, 360.0f));
                }
                // サーボを逆(左: 180度)に旋回させて横の物体を向き続ける
                if (SENSOR_SERVO_ENABLED) {
                    servoDrive.moveServo(ServoConfig::SENSOR, 180.0f);
                }
                changeAutoState(AUTO_TURNING_RIGHT);
            }
            break;
        }

        case AUTO_TURNING_RIGHT: {
            gait.setVelocity(0, 0, -TURN_SPEED);

            if (imuPid.isReady()) {
                float targetHeading = fmodf(_targetHeadingSave + 90.0f + 360.0f, 360.0f);
                float currentHeading = imuPid.getHeadingDeg();

                float diff = fabsf(currentHeading - targetHeading);
                if (diff > 180.0f) diff = 360.0f - diff;

                static unsigned long lastTurnLog = 0;
                if (millis() - lastTurnLog > 200) {
                    Serial.print(F("[TURN_R] H:")); Serial.print(currentHeading, 1);
                    Serial.print(F(" T:")); Serial.print(targetHeading, 1);
                    Serial.print(F(" D:")); Serial.println(diff, 1);
                    lastTurnLog = millis();
                }

                unsigned long elapsed = millis() - _stateStartTime;
                if ((elapsed > 1500 && diff <= 20.0f) || elapsed > 8000) {
                    Serial.print(F("[Auto] Right Turn Done! H:"));
                    Serial.print(currentHeading, 1);
                    Serial.print(F(" D:")); Serial.print(diff, 1);
                    Serial.print(F(" t:")); Serial.println(elapsed);
                    imuPid.setTargetDeg(targetHeading);
                    gait.setVelocity(MOVE_SPEED, 0, 0);
                    changeAutoState(AUTO_TRACKING_OBJECT);
                }
            } else {
                if (millis() - _stateStartTime >= 3000) {
                    Serial.println(F("[Auto] Right Turn Done (Timer)."));
                    gait.setVelocity(MOVE_SPEED, 0, 0);
                    changeAutoState(AUTO_TRACKING_OBJECT);
                }
            }
            break;
        }

        case AUTO_TRACKING_OBJECT: {
            gait.setVelocity(MOVE_SPEED, 0, 0);

            if (!SENSOR_SERVO_ENABLED) {
                if (millis() - _stateStartTime > AUTO_FORWARD_SIDE_MS) {
                    Serial.print(F("[Auto] Sidewalk done ("));
                    Serial.print(AUTO_FORWARD_SIDE_MS);
                    Serial.println(F("ms)! Left turn back..."));

                    gait.setVelocity(0, 0, TURN_SPEED);
                    if (imuPid.isReady()) {
                        imuPid.setTargetDeg(_targetHeadingSave);
                    }
                    changeAutoState(AUTO_TURNING_BACK);
                }
            } else {
                if (millis() - _stateStartTime > 1000) {
                    if (_lastDistance < 0 || _lastDistance > OBSTACLE_CLEARANCE_DIST) {
                        Serial.println(F("[Auto] Side edge cleared!"));
                        servoDrive.moveServo(ServoConfig::SENSOR, 135.0f);
                        changeAutoState(AUTO_CLEARING_MARGIN);
                    }
                }
            }
            break;
        }

        case AUTO_CLEARING_MARGIN: {
            if (!SENSOR_SERVO_ENABLED) {
                gait.setVelocity(0, 0, 0);
                if (millis() - _stateStartTime < 1500) break;

                if (_lastDistance > 0 && _lastDistance <= OBSTACLE_DETECTION_DIST) {
                    Serial.print(F("[Auto] Still blocked ("));
                    Serial.print(_lastDistance, 1);
                    Serial.println(F("cm)! Re-turn Right..."));

                    gait.setVelocity(0, 0, -TURN_SPEED);
                    if (imuPid.isReady()) {
                        imuPid.setTargetDeg(fmodf(_targetHeadingSave + 90.0f + 360.0f, 360.0f));
                    }
                    changeAutoState(AUTO_TURNING_RIGHT);
                } else {
                    Serial.println(F("[Auto] Front clear! Final forward..."));
                    if (imuPid.isReady()) {
                        imuPid.setTargetDeg(_targetHeadingSave);
                    }
                    gait.setVelocity(MOVE_SPEED, 0, 0);
                    changeAutoState(AUTO_FORWARD_FINAL);
                }
            } else {
                gait.setVelocity(MOVE_SPEED, 0, 0);
                if (millis() - _stateStartTime > 300) {
                    if (_lastDistance < 0 || _lastDistance > OBSTACLE_CLEARANCE_DIST) {
                        Serial.println(F("[Auto] Rear clear! Left turn back..."));

                        gait.setVelocity(0, 0, TURN_SPEED);
                        if (imuPid.isReady()) {
                            imuPid.setTargetDeg(_targetHeadingSave);
                        }
                        servoDrive.moveServo(ServoConfig::SENSOR, 90.0f);
                        changeAutoState(AUTO_TURNING_BACK);
                    }
                }
            }
            break;
        }

        case AUTO_TURNING_BACK: {
            gait.setVelocity(0, 0, TURN_SPEED);

            if (imuPid.isReady()) {
                float targetHeading = _targetHeadingSave;
                float currentHeading = imuPid.getHeadingDeg();

                float diff = fabsf(currentHeading - targetHeading);
                if (diff > 180.0f) diff = 360.0f - diff;

                static unsigned long lastBackLog = 0;
                if (millis() - lastBackLog > 200) {
                    Serial.print(F("[TURN_L] H:")); Serial.print(currentHeading, 1);
                    Serial.print(F(" T:")); Serial.print(targetHeading, 1);
                    Serial.print(F(" D:")); Serial.println(diff, 1);
                    lastBackLog = millis();
                }

                unsigned long elapsed = millis() - _stateStartTime;
                if ((elapsed > 1500 && diff <= 20.0f) || elapsed > 8000) {
                    Serial.print(F("[Auto] Left Turn Done! H:"));
                    Serial.print(currentHeading, 1);
                    Serial.print(F(" D:")); Serial.print(diff, 1);
                    Serial.print(F(" t:")); Serial.println(elapsed);
                    imuPid.setTargetDeg(_targetHeadingSave);
                    gait.setVelocity(0, 0, 0);
                    changeAutoState(AUTO_CLEARING_MARGIN);
                }
            } else {
                if (millis() - _stateStartTime >= 4000) {
                    Serial.println(F("[Auto] Left Turn Done (Timer)."));
                    gait.setVelocity(0, 0, 0);
                    changeAutoState(AUTO_CLEARING_MARGIN);
                }
            }
            break;
        }

        case AUTO_FORWARD_FINAL: {
            gait.setVelocity(MOVE_SPEED, 0, 0);
            if (millis() - _stateStartTime >= AUTO_FORWARD_FINAL_MS) {
                Serial.println(F("[Auto] Final forward complete! Mission Complete. Stopping..."));
                gait.setVelocity(0, 0, 0);
                gait.stand();
                changeAutoState(AUTO_FINISHED);
            }
            break;
        }

        case AUTO_FINISHED: {
            gait.setVelocity(0, 0, 0);
            break;
        }

        default:
            break;
    }
}

const char* Commander::getAutoStateName(AutoState state) {
    switch (state) {
        case AUTO_IDLE:            return "IDLE";
        case AUTO_FORWARD_INIT:    return "FORWARD_INIT";
        case AUTO_TURNING_RIGHT:   return "TURNING_RIGHT";
        case AUTO_TRACKING_OBJECT: return "TRACKING_OBJECT";
        case AUTO_CLEARING_MARGIN: return "CLEARING_MARGIN";
        case AUTO_TURNING_BACK:    return "TURNING_BACK";
        case AUTO_FORWARD_FINAL:   return "FORWARD_FINAL";
        case AUTO_FINISHED:        return "FINISHED";
        default:                   return "UNKNOWN";
    }
}

void Commander::changeAutoState(AutoState newState) {
    if (_autoState != newState) {
        Serial.print(F("[AutoState] Transition: "));
        Serial.print(getAutoStateName(_autoState));
        Serial.print(F(" -> "));
        Serial.println(getAutoStateName(newState));
        
        _autoState = newState;
        _stateStartTime = millis();
    }
}

void Commander::updateForwardDetectStop() {
    if (!_forwardDetectStopMode) return;

    if (_lastDistance > 0 && _lastDistance <= ControlConfig::OBSTACLE_DETECTION_DIST) {
        Serial.print(F("[ForwardDetectStop] Obstacle detected within "));
        Serial.print(ControlConfig::OBSTACLE_DETECTION_DIST, 1);
        Serial.println(F("cm! Stopping robot..."));

        gait.setVelocity(0, 0, 0);
        gait.stand();
        _forwardDetectStopMode = false;
    } else {
        gait.setVelocity(ControlConfig::MOVE_SPEED, 0, 0);
    }
}

