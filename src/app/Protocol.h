#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <Arduino.h>
#include "reaction_Esp.h"

message_t makeRegisterMessage(uint8_t sourceLightId, uint8_t groupId);
message_t makeRegisterAckMessage(uint8_t targetLightId, uint8_t groupId);
message_t makeStateSetMessage(uint8_t stateId, uint8_t groupId);
message_t makeSensorEventMessage(uint8_t stateId, uint8_t groupId, uint8_t sourceLightId, uint8_t eventId, uint32_t value);
message_t makeLightStateMessage(uint8_t stateId, uint8_t groupId, uint8_t targetLightId, uint16_t red, uint16_t green, uint16_t blue);
bool isExpectedMessageSize(uint8_t len);
bool isValidMessageForGroup(const message_t* message, uint8_t localGroupId);

#endif
