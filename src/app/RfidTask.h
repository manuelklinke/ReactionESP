#ifndef APP_RFID_TASK_H
#define APP_RFID_TASK_H

#include <Arduino.h>
#include <Adafruit_PN532.h>
#include "reaction_Esp.h"

uint8_t stateIdForRfidUid(const uint8_t* uid, uint8_t uidLength);
bool pollRfidForState(Adafruit_PN532* nfc, uint8_t* stateId);

#endif
