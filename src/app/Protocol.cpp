#include "app/Protocol.h"

static message_t makeBaseMessage(uint8_t messageType, uint8_t stateId, uint8_t groupId) {
    message_t message = {};
    message.messageType = messageType;
    message.stateId = stateId;
    message.groupId = groupId;
    message.sourceLightId = UNKNOWN_LIGHT_ID;
    message.targetLightId = UNKNOWN_LIGHT_ID;
    message.eventId = EVENT_NONE;
    return message;
}

message_t makeRegisterMessage(uint8_t sourceLightId, uint8_t groupId) {
    message_t message = makeBaseMessage(MSG_REGISTER, STATE_INIT, groupId);
    message.sourceLightId = sourceLightId;
    return message;
}

message_t makeRegisterAckMessage(uint8_t targetLightId, uint8_t groupId) {
    message_t message = makeBaseMessage(MSG_REGISTER_ACK, STATE_INIT, groupId);
    message.targetLightId = targetLightId;
    return message;
}

message_t makeStateSetMessage(uint8_t stateId, uint8_t groupId) {
    return makeBaseMessage(MSG_STATE_SET, stateId, groupId);
}

message_t makeSensorEventMessage(uint8_t stateId, uint8_t groupId, uint8_t sourceLightId, uint8_t eventId, uint32_t value) {
    message_t message = makeBaseMessage(MSG_SENSOR_EVENT, stateId, groupId);
    message.sourceLightId = sourceLightId;
    message.eventId = eventId;
    message.value = value;
    return message;
}

message_t makeLightStateMessage(uint8_t stateId, uint8_t groupId, uint8_t targetLightId, uint16_t red, uint16_t green, uint16_t blue) {
    message_t message = makeBaseMessage(MSG_LIGHT_STATE, stateId, groupId);
    message.targetLightId = targetLightId;
    message.color_r = red;
    message.color_g = green;
    message.color_b = blue;
    return message;
}

bool isExpectedMessageSize(uint8_t len) {
    return len == sizeof(message_t);
}

bool isValidMessageForGroup(const message_t* message, uint8_t localGroupId) {
    return message != nullptr && messageMatchesGroup(localGroupId, message->groupId);
}
