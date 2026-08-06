#include "Gait.h"
#include <math.h>
#include "Commander.h"

GaitManager gait;

void Leg::init(int hipCh, int kneeCh, float xOffset, float yOffset) {
    _hipCh = hipCh;
    _kneeCh = kneeCh;
    _xOffset = xOffset;
    _yOffset = yOffset;

    _baseAngle = atan2(
        getHomeY() - _yOffset,
        getHomeX() - _xOffset
    );

    // LERP減衰定数
    _lerpK = -log(1.0f - GaitConfig::LERP_RATE) * 60.0f;

    currentPos = {getHomeX(), getHomeY(), -GaitConfig::DEFAULT_Z};
    _targetPos = currentPos;
}

void Leg::setTarget(Point3D p) {
    _targetPos = p;
}

void Leg::goHome() {
    setTarget({getHomeX(), getHomeY(), -GaitConfig::DEFAULT_Z});
}

void Leg::update(float dt) {
    // 2次Padé近似で指数減衰を計算
    float kdt = _lerpK * dt;
    float alpha = 1.0f - 1.0f / (1.0f + kdt + 0.5f * kdt * kdt);

    alpha = constrain(alpha, 0.001f, 1.0f);

    currentPos.x += (_targetPos.x - currentPos.x) * alpha;
    currentPos.y += (_targetPos.y - currentPos.y) * alpha;
    currentPos.z += (_targetPos.z - currentPos.z) * alpha;

    solveIK(currentPos);
}

void Leg::solveIK(Point3D p) {
    float lx = p.x - _xOffset;
    float ly = p.y - _yOffset;

    float reach = hypotf(lx, ly);
    if (reach < 1.0f) {
        lx = 1.0f;
    }

    // 1. Hip IK
    float homeLx = getHomeX() - _xOffset;
    float homeLy = getHomeY() - _yOffset;
    float cross = homeLy * lx - homeLx * ly;
    float dot   = homeLx * lx + homeLy * ly;
    float diffAngle = atan2f(cross, dot);

    // 2. Knee IK
    float targetZ = fabsf(p.z);

    float sinKnee = targetZ / RobotConfig::L2;
    if (sinKnee > 1.0f) sinKnee = 1.0f;
    if (sinKnee < 0.0f) sinKnee = 0.0f;

    float thetaKnee = asin(sinKnee) * 180.0f / M_PI;
    float thetaHip  = diffAngle * 180.0f / M_PI;

    servoDrive.moveServo(_hipCh,  ServoConfig::NEUTRAL_ANGLE + thetaHip);
    servoDrive.moveServo(_kneeCh, 180.0f - thetaKnee);
}

GaitManager::GaitManager()
    : _vx(0), _vy(0), _vtheta(0),
      _startTime(0), _lastUpdateTime(0), _savedPhase(0),
      _moving(false), _wasMoving(false), _shiftDisabled(true),
      _stepHeight(GaitConfig::STEP_HEIGHT),
      _stepLength(GaitConfig::STEP_LENGTH),
      _cycleTime(GaitConfig::CYCLE_TIME) {}

void GaitManager::begin() {
    using namespace RobotConfig;
    using namespace ServoConfig;

    _legs[0].init(FR_HIP, FR_KNEE, FR_OFFSET_X, FR_OFFSET_Y);
    _legs[1].init(BL_HIP, BL_KNEE, BL_OFFSET_X, BL_OFFSET_Y);
    _legs[2].init(FL_HIP, FL_KNEE, FL_OFFSET_X, FL_OFFSET_Y);
    _legs[3].init(BR_HIP, BR_KNEE, BR_OFFSET_X, BR_OFFSET_Y);

    _startTime = _lastUpdateTime = millis();
}

void GaitManager::setVelocity(float vx, float vy, float vtheta) {
    _vx = vx; _vy = vy; _vtheta = vtheta;
    _moving = (fabsf(vx) > 0.1f || fabsf(vy) > 0.1f || fabsf(vtheta) > 0.1f);
}

void GaitManager::stop() { _moving = false; }

void GaitManager::stand() {
    stop();
    if (imuPid.isReady()) {
        imuPid.captureTarget();
    }
    for (int i = 0; i < GaitConfig::LEG_COUNT; i++) {
        _legs[i].goHome();
    }
}

void GaitManager::update() {
    unsigned long now = millis();
    float dt = (now - _lastUpdateTime) * 0.001f;
    _lastUpdateTime = now;

    if (dt > 0.1f)   dt = 0.1f;
    if (dt < 0.001f) dt = 0.001f;

    if (_moving && !_wasMoving) {
        _startTime = now - (unsigned long)(_savedPhase * GaitConfig::CYCLE_TIME);
        if (imuPid.isReady()) {
            imuPid.captureTarget();
        }
    }

    // IMU PID補正
    float vthetaEff = _vtheta;
    if (ImuPidConfig::ENABLED && imuPid.isReady() && _moving) {
        bool isTurning = fabsf(_vtheta) > ImuPidConfig::TURN_DEAD;
        if (isTurning) {
            if (!commander.isAutoDriveMode()) {
                imuPid.captureTarget();
            }
        } else {
            vthetaEff += imuPid.compute(dt);
        }
    }

    if (_moving) {
        calculateGait(now, vthetaEff);
    } else if (_wasMoving) {
        unsigned long t = now - _startTime;
        _savedPhase = (float)(t % GaitConfig::CYCLE_TIME) / GaitConfig::CYCLE_TIME;
    }
    _wasMoving = _moving;

    for (int i = 0; i < GaitConfig::LEG_COUNT; i++) {
        _legs[i].update(dt);
    }
}

void GaitManager::calculateGait(unsigned long now, float vthetaEff) {
    unsigned long t = now - _startTime;
    float phase = (float)(t % _cycleTime) / _cycleTime;

    // 重心移動 (Body Shifting)
    float maxCmd = fmaxf(fmaxf(fabsf(_vx), fabsf(_vy)), fabsf(vthetaEff) * (ControlConfig::MOVE_SPEED / fmaxf(0.1f, ControlConfig::TURN_SPEED)));
    float speedRatio = (ControlConfig::MOVE_SPEED > 0.0f) ? (maxCmd / ControlConfig::MOVE_SPEED) : 0.0f;
    speedRatio = constrain(speedRatio, 0.0f, 1.0f);

    float shiftPhase = fmod(phase + 0.05f, 1.0f);
    
    bool isPureLeftTurn = (vthetaEff > 0.05f && fabsf(_vx) < 0.1f && fabsf(_vy) < 0.1f);
    float gaitDir = isPureLeftTurn ? -1.0f : 1.0f;

    float shiftAngle = gaitDir * shiftPhase * 2.0f * M_PI + M_PI / 4.0f;
    float bodyOffsetX = -sin(shiftAngle) * (GaitConfig::BODY_SHIFT_X * 1.414f * speedRatio);
    float bodyOffsetY = -cos(shiftAngle) * (GaitConfig::BODY_SHIFT_Y * 1.414f * speedRatio);

    if (_shiftDisabled) {
        bodyOffsetX = 0.0f;
        bodyOffsetY = 0.0f;
    }

    float headingRad = GaitConfig::HEADING_ANGLE * M_PI / 180.0f;
    float cosH = cos(headingRad);
    float sinH = sin(headingRad);
    float rotVx = _vx * cosH - _vy * sinH;
    float rotVy = _vx * sinH + _vy * cosH;

    float strideX   = rotVx    * (_stepLength / GaitConfig::STRIDE_X_DIV);
    float strideY   = rotVy    * (_stepLength / GaitConfig::STRIDE_Y_DIV);
    float strideRot = vthetaEff * (M_PI / GaitConfig::STRIDE_ROT_DIV);

    // 脚の接地・離陸タイミング（クロール歩行）
    float phaseOffsets[4];
    if (isPureLeftTurn) {
        phaseOffsets[0] = 0.0f;
        phaseOffsets[1] = 0.5f;
        phaseOffsets[2] = 0.75f;
        phaseOffsets[3] = 0.25f;
    } else {
        phaseOffsets[0] = 0.0f;
        phaseOffsets[1] = 0.5f;
        phaseOffsets[2] = 0.25f;
        phaseOffsets[3] = 0.75f;
    }
    
    for (int i = 0; i < GaitConfig::LEG_COUNT; i++) {
        float legPhase = fmod(phase - phaseOffsets[i] + 1.0f, 1.0f);
        float homeX    = _legs[i].getHomeX();
        float homeY    = _legs[i].getHomeY();

        float s, targetZ;
        bool isSwing = (legPhase < GaitConfig::SWING_RATIO);

        if (isSwing) {
            // Swing Phase (遊脚)
            s = legPhase / GaitConfig::SWING_RATIO;
            float h = sin(s * M_PI);
            targetZ = -GaitConfig::DEFAULT_Z + h * h * _stepHeight;
        } else {
            // Stance Phase (支持脚)
            s       = (legPhase - GaitConfig::SWING_RATIO) / (1.0f - GaitConfig::SWING_RATIO);
            targetZ = -GaitConfig::DEFAULT_Z - GaitConfig::STANCE_PULLDOWN_Z;
        }

        float sign    = isSwing ? 1.0f : -1.0f;
        float sCenter = sign * (s - 0.5f);

        float targetX = homeX + strideX * sCenter - homeY * strideRot * sCenter - bodyOffsetX;
        float targetY = homeY + strideY * sCenter + homeX * strideRot * sCenter - bodyOffsetY;

        _legs[i].setTarget({targetX, targetY, targetZ});
    }
}
