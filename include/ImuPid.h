#ifndef IMU_PID_H
#define IMU_PID_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Config.h"

class ImuPid {
public:
    ImuPid();

    bool begin();
    void captureTarget();
    float compute(float dt);
    float getHeadingDeg();

    float getTargetDeg() const { return _targetHeading; }

    void setTargetDeg(float target) {
        _targetHeading = fmodf(target + 360.0f, 360.0f);
        _integral  = 0.0f;
        _prevError = 0.0f;
    }

    bool isReady() const { return _ready; }

    void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
        if (_ready) {
            _bno.getCalibration(sys, gyro, accel, mag);
        } else {
            *sys = *gyro = *accel = *mag = 0;
        }
    }

private:
    Adafruit_BNO055 _bno;

    float _targetHeading;
    float _integral;
    float _prevError;
    bool  _ready;

    static float _normalizeError(float deg);
};

extern ImuPid imuPid;

#endif
