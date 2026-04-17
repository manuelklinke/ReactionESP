#include <Arduino.h>

#include "reaction_Esp.h"

void setup() {
    Serial.begin(115200);

    bool ok = true;
    ok = ok && (groupFromSwitch(0) == GROUP_A);
    ok = ok && (groupFromSwitch(1) == GROUP_B);
    ok = ok && messageMatchesGroup(GROUP_A, GROUP_A);
    ok = ok && messageMatchesGroup(GROUP_B, GROUP_B);
    ok = ok && messageMatchesGroup(GROUP_A, GROUP_ALL);
    ok = ok && !messageMatchesGroup(GROUP_A, GROUP_B);
    ok = ok && !messageMatchesGroup(GROUP_B, GROUP_A);

    Serial.println(ok ? "group routing test passed" : "group routing test failed");
}

void loop() {
}
