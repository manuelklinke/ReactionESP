#include <Arduino.h>
#include <stddef.h>
#include <type_traits>
#include "app/NodeConfig.h"
#include "app/Protocol.h"
#include "app/RfidTask.h"
#include "states/InitState.h"
#include "states/StateRegistry.h"
#include "reaction_Esp.h"

static_assert(MAX_LIGHTS == 8, "MAX_LIGHTS must remain 8");
static_assert(GROUP_A == 0, "GROUP_A must be 0");
static_assert(GROUP_B == 1, "GROUP_B must be 1");
static_assert(GROUP_ALL == 255, "GROUP_ALL must be 255");
static_assert(STATE_INIT == 0, "Init state must be available before any game state");
static_assert(STATE_CLASSIC_1_OF_N == 1, "Classic state id must be 1");
static_assert(MSG_REGISTER == 1, "Register message type must be 1");
static_assert(MSG_STATE_SET == 3, "State-set message type must be 3");
static_assert(EVENT_TAP == 1, "Tap event id must be 1");
static_assert(EVENT_SQUEEZE == 6, "Squeeze event id must be 6");
static_assert(EVENT_UNSQUEEZE == 7, "Unsqueeze event id must be 7");
static_assert(sizeof(message_t) <= 64, "ESP-NOW message must stay compact");
static_assert(offsetof(message_t, messageType) == 0, "messageType must be first");
static_assert(offsetof(message_t, stateId) == 1, "stateId offset must remain stable");
static_assert(offsetof(message_t, groupId) == 2, "groupId offset must remain stable");
static_assert(offsetof(message_t, sourceLightId) == 4, "sourceLightId offset must remain stable");
static_assert(offsetof(message_t, targetLightId) == 5, "targetLightId offset must remain stable");
static_assert(offsetof(message_t, eventId) == 6, "eventId offset must remain stable");
static_assert(offsetof(message_t, color_r) >= 8, "color_r must remain present");
static_assert(offsetof(message_t, value) > offsetof(message_t, touched), "value must remain present after flags");
static_assert(std::is_same<typename std::remove_reference<decltype(((message_t *)0)->messageType)>::type, uint8_t>::value, "messageType must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((message_t *)0)->stateId)>::type, uint8_t>::value, "stateId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((message_t *)0)->groupId)>::type, uint8_t>::value, "groupId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((message_t *)0)->sourceLightId)>::type, uint8_t>::value, "sourceLightId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((message_t *)0)->targetLightId)>::type, uint8_t>::value, "targetLightId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((message_t *)0)->eventId)>::type, uint8_t>::value, "eventId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((message_t *)0)->color_r)>::type, uint16_t>::value, "color_r must be uint16_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((message_t *)0)->value)>::type, uint32_t>::value, "value must be uint32_t");
static_assert(sizeof(((Ctx *)0)->registeredLights) == MAX_LIGHTS, "registeredLights must cover MAX_LIGHTS");
static_assert(offsetof(Ctx, isMaster) == 0, "isMaster must be first");
static_assert(offsetof(Ctx, role) == 1, "role must follow isMaster");
static_assert(offsetof(Ctx, lightId) == 2, "lightId must remain present");
static_assert(offsetof(Ctx, groupId) == 3, "groupId must remain present");
static_assert(offsetof(Ctx, stateId) == 4, "stateId must remain present");
static_assert(offsetof(Ctx, requestedStateId) == 5, "requestedStateId must remain present");
static_assert(offsetof(Ctx, activeLightId) == 6, "activeLightId must remain present");
static_assert(offsetof(Ctx, registeredLights) == 7, "registeredLights must remain present");
static_assert(offsetof(Ctx, light) > offsetof(Ctx, registeredLights), "light array must remain present");
static_assert(std::is_same<typename std::remove_reference<decltype(((Ctx *)0)->isMaster)>::type, uint8_t>::value, "isMaster must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((Ctx *)0)->role)>::type, uint8_t>::value, "role must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((Ctx *)0)->lightId)>::type, uint8_t>::value, "lightId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((Ctx *)0)->stateId)>::type, uint8_t>::value, "stateId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((Ctx *)0)->requestedStateId)>::type, uint8_t>::value, "requestedStateId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((Ctx *)0)->activeLightId)>::type, uint8_t>::value, "activeLightId must be uint8_t");
static_assert(std::is_same<typename std::remove_reference<decltype(((Ctx *)0)->timerValue)>::type, uint64_t>::value, "timerValue must be uint64_t");

void setup() {
    Serial.begin(115200);
    message_t msg = {};
    bool registryOk = stateEnterFunctionFor(STATE_INIT, ROLE_MASTER) != nullptr &&
                      stateEnterFunctionFor(STATE_CLASSIC_1_OF_N, ROLE_MASTER) != nullptr &&
                      stateEnterFunctionFor(STATE_CLASSIC_1_OF_N, ROLE_LIGHT) != nullptr;
    Serial.println(registryOk ? "state registry test passed" : "state registry test failed");
    uint8_t masterMac[6] = {0xCC, 0x7B, 0x5C, 0x4F, 0xD2, 0x84};
    uint8_t firstSlaveMac[6] = {0xD8, 0xBF, 0xC0, 0x17, 0xA6, 0x9A};
    bool lookupOk = (lightIdByMacAddress(masterMac) == 0) && (lightIdByMacAddress(firstSlaveMac) == 1);
    Serial.println(lookupOk ? "node lookup test passed" : "node lookup test failed");
    msg.messageType = MSG_SENSOR_EVENT;
    msg.stateId = STATE_CLASSIC_1_OF_N;
    msg.groupId = GROUP_A;
    msg.sourceLightId = 2;
    msg.targetLightId = 2;
    msg.eventId = EVENT_TAP;
    Serial.println(messageMatchesGroup(GROUP_A, msg.groupId) ? "protocol compile test passed" : "protocol compile test failed");
    message_t registerMsg = makeRegisterMessage(3, GROUP_B);
    bool registerOk = registerMsg.messageType == MSG_REGISTER &&
                      registerMsg.sourceLightId == 3 &&
                      registerMsg.groupId == GROUP_B &&
                      isValidMessageForGroup(&registerMsg, GROUP_B);

    message_t stateMsg = makeStateSetMessage(STATE_CLASSIC_1_OF_N, GROUP_A);
    bool stateOk = stateMsg.messageType == MSG_STATE_SET &&
                   stateMsg.stateId == STATE_CLASSIC_1_OF_N &&
                   stateMsg.groupId == GROUP_A;

    message_t ack = makeRegisterAckMessage(2, GROUP_A);
    bool ackOk = ack.messageType == MSG_REGISTER_ACK && ack.targetLightId == 2 && ack.groupId == GROUP_A;
    Serial.println(ackOk ? "register ack test passed" : "register ack test failed");

    message_t squeezeMsg = makeSensorEventMessage(STATE_CLASSIC_1_OF_N, GROUP_A, 2, EVENT_SQUEEZE, 1);
    bool squeezeOk = squeezeMsg.messageType == MSG_SENSOR_EVENT &&
                     squeezeMsg.eventId == EVENT_SQUEEZE &&
                     squeezeMsg.sourceLightId == 2 &&
                     squeezeMsg.value == 1;
    Serial.println(squeezeOk ? "squeeze event test passed" : "squeeze event test failed");

    message_t unsqueezeMsg = makeSensorEventMessage(STATE_CLASSIC_1_OF_N, GROUP_A, 2, EVENT_UNSQUEEZE, 0);
    bool unsqueezeOk = unsqueezeMsg.messageType == MSG_SENSOR_EVENT &&
                       unsqueezeMsg.eventId == EVENT_UNSQUEEZE &&
                       unsqueezeMsg.sourceLightId == 2 &&
                       unsqueezeMsg.value == 0;
    Serial.println(unsqueezeOk ? "unsqueeze event test passed" : "unsqueeze event test failed");

    uint8_t uid[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    bool rfidOk = stateIdForRfidUid(nullptr, 0) == STATE_INIT &&
                  stateIdForRfidUid(uid, sizeof(uid)) == STATE_CLASSIC_1_OF_N;
    Serial.println(rfidOk ? "rfid state test passed" : "rfid state test failed");

    Serial.println(registerOk && stateOk ? "protocol helper test passed" : "protocol helper test failed");
}

void loop() {
}
