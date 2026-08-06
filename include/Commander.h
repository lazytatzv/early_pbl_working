#ifndef COMMANDER_H
#define COMMANDER_H

#include <Arduino.h>

class Commander {
public:
    Commander();
    void begin();
    void update();

    bool isAutoDriveMode() const { return _autoDriveMode; }

private:
    void processSerialInput();
    void checkInputTimeout();
    void updateSensors();
    void runDiagnostic();
    void runUltrasonicTest();
    void runImuTest();
    void updateAutoDrive();
    void printStatus();

    enum AutoState {
        AUTO_IDLE,
        AUTO_FORWARD_INIT,
        AUTO_TURNING_RIGHT,
        AUTO_TRACKING_OBJECT,
        AUTO_CLEARING_MARGIN,
        AUTO_TURNING_BACK,
        AUTO_FORWARD_FINAL,
        AUTO_FINISHED
    };

    const char* getAutoStateName(AutoState state);
    void changeAutoState(AutoState newState);

    unsigned long _lastCommandTime;
    unsigned long _lastSensorCheck;
    unsigned long _lastLogTime;
    float _lastDistance;
    bool _ultrasonicTestMode;
    bool _imuTestMode;
    bool _autoDriveMode;
    bool _forwardDetectStopMode;
    bool _roughTerrainMode;
    float _savedNormalStepHeight;
    int _savedNormalCycleTime;
    AutoState _autoState;
    unsigned long _stateStartTime;
    float _targetHeadingSave;
    void updateForwardDetectStop();
};

extern Commander commander;

#endif
