#include <Arduino.h>
#include "states/classic1ofn/Classic1OfNState.h"

void setup() {
    Serial.begin(115200);

    uint8_t registered[MAX_LIGHTS] = {};
    registered[1] = 1;
    registered[2] = 1;
    registered[3] = 1;

    uint8_t nextFromMaster = classicChooseNextLight(0, registered, MAX_LIGHTS, 0);
    uint8_t nextFrom1 = classicChooseNextLight(1, registered, MAX_LIGHTS, 0);
    uint8_t nextFrom2 = classicChooseNextLight(2, registered, MAX_LIGHTS, 1);
    uint8_t nextFrom3 = classicChooseNextLight(3, registered, MAX_LIGHTS, 2);

    bool ok = nextFromMaster != 0 &&
              nextFrom1 != 1 &&
              nextFrom2 != 2 &&
              nextFrom3 != 3 &&
              nextFromMaster < MAX_LIGHTS &&
              nextFrom1 < MAX_LIGHTS &&
              nextFrom2 < MAX_LIGHTS &&
              nextFrom3 < MAX_LIGHTS;

    uint8_t registeredWithMaster[MAX_LIGHTS] = {};
    registeredWithMaster[0] = 1;
    registeredWithMaster[1] = 1;
    bool masterCanBeChosen = classicChooseNextLight(1, registeredWithMaster, MAX_LIGHTS, 0) == 0;

    uint8_t singleRegistered[MAX_LIGHTS] = {};
    singleRegistered[4] = 1;
    bool singleOk = classicChooseNextLight(4, singleRegistered, MAX_LIGHTS, 99) == 4;

    Serial.println(ok && masterCanBeChosen && singleOk ? "classic target test passed" : "classic target test failed");
}

void loop() {
}
