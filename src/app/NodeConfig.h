#ifndef APP_NODE_CONFIG_H
#define APP_NODE_CONFIG_H

#include <Arduino.h>
#include "reaction_Esp.h"

#define PIN_NEOPIXEL 0
#define PIN_STATUS_LED 2
#define PIN_GROUP_SELECT 14
#define PIN_TOUCH 15
#define PIN_MASTER_SELECT 16
#define PIN_ADXL_INTERRUPT 13
#define PIN_PN532_INTERRUPT 12

#define ESPNOW_CHANNEL 1
#define STATIC_SLAVE_COUNT 3

extern uint8_t MasterAddress[6];
extern uint8_t SlaveAddress[STATIC_SLAVE_COUNT][6];

uint8_t roleFromMasterSwitch(uint8_t switchHigh);
uint8_t lightIdByMacAddress(const uint8_t* macAddr);
bool copyPeerAddressForLight(uint8_t lightId, uint8_t* destination);

#endif
