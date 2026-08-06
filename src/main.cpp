#include <Arduino.h>
#include "Commander.h"
#include "Gait.h"

void setup() {
    Serial.begin(115200);
    while (!Serial); 

    Serial.println(F("========================================"));
    Serial.println(F("   Quadruped Robot System v" FIRMWARE_VERSION));
    Serial.println(F("========================================"));

    // 各システムの起動
    commander.begin();
    
    Serial.println(F("[Info] Boot Sequence Complete. Standing up..."));
    gait.stand();
    
    Serial.println(F("[Usage] HOLD WASD/JK to move, 'r' to reset, 't' to test."));
}

void loop() {
    gait.update();
    commander.update();
}
