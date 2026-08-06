#ifndef GAIT_H
#define GAIT_H

#include <Arduino.h>
#include "ServoDrive.h"
#include "Config.h"
#include "ImuPid.h"

struct Point3D {
    float x, y, z;
};

class Leg {
public:
    Leg() : _xOffset(0), _yOffset(0), _baseAngle(0) {}
    
    void init(int hipCh, int kneeCh, float xOffset, float yOffset);
    void setTarget(Point3D p);
    void update(float dt);
    void goHome();

    float getOffsetX() const { return _xOffset; }
    float getOffsetY() const { return _yOffset; }

    float getHomeX() const {
        return _xOffset + (_xOffset > 0 ? RobotConfig::STANCE_OFFSET_X : -RobotConfig::STANCE_OFFSET_X);
    }
    float getHomeY() const {
        return _yOffset + (_yOffset > 0 ? RobotConfig::STANCE_OFFSET_Y : -RobotConfig::STANCE_OFFSET_Y);
    }

    Point3D currentPos;
    
private:
    int _hipCh, _kneeCh;
    float _xOffset, _yOffset;
    float _baseAngle; // 脚の基準取付角度 (radian)
    float _lerpK;     // 時間ベースLERPの減衰定数 (init()で1回だけ計算)
    Point3D _targetPos;

    void solveIK(Point3D p);
};

class GaitManager {
public:
    GaitManager();
    void begin();
    void update();

    void setVelocity(float vx, float vy, float vtheta);
    void stop();
    void stand();

    void setShiftDisabled(bool disabled) { _shiftDisabled = disabled; }
    bool isShiftDisabled() const { return _shiftDisabled; }

    void adjustStepHeight(float delta) { _stepHeight += delta; }
    void adjustStepLength(float delta) { _stepLength += delta; }
    void adjustCycleTime(int delta) { _cycleTime += delta; if (_cycleTime < 100) _cycleTime = 100; }
    void setStepHeight(float val) { _stepHeight = val; }
    void setCycleTime(int val) { _cycleTime = val; if (_cycleTime < 100) _cycleTime = 100; }
    float getStepHeight() const { return _stepHeight; }
    float getStepLength() const { return _stepLength; }
    int getCycleTime() const { return _cycleTime; }

private:
    Leg _legs[4];
    float _vx, _vy, _vtheta;
    unsigned long _startTime;
    unsigned long _lastUpdateTime;
    float _savedPhase;
    bool _moving;
    bool _wasMoving;
    bool _shiftDisabled;

    float _stepHeight;
    float _stepLength;
    int _cycleTime;

    void calculateGait(unsigned long now, float vthetaEff);
};

extern GaitManager gait;

#endif
