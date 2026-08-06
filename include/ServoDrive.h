#ifndef SERVO_DRIVE_H
#define SERVO_DRIVE_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Config.h"

struct JointCalibration {
    float offset;
    int direction;
    float minAng;
    float maxAng;
};

class ServoDrive {
public:
    ServoDrive();
    void begin();

    void moveServo(int ch, float angle);

private:
    Adafruit_PWMServoDriver pwm;
    JointCalibration _calibs[16];
    int _lastPulses[16];

    void _initCalibrations();
};

extern ServoDrive servoDrive;

#endif
