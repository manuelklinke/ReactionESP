#ifndef REACTION_ESP_H_
#define REACTION_ESP_H_

#include "Arduino.h"

#define MAX_LIGHTS 8
#define PRESSURE_OFFSET 100
#define GROUP_A 0
#define GROUP_B 1
#define GROUP_ALL 255
#define UNKNOWN_LIGHT_ID 255
#define NO_ACTIVE_LIGHT 255

enum StateId : uint8_t {
    STATE_INIT = 0,
    STATE_CLASSIC_1_OF_N = 1
};

enum MessageType : uint8_t {
    MSG_NONE = 0,
    MSG_REGISTER = 1,
    MSG_REGISTER_ACK = 2,
    MSG_STATE_SET = 3,
    MSG_SENSOR_EVENT = 4,
    MSG_LIGHT_STATE = 5,
    MSG_HEARTBEAT = 6
};

enum EventId : uint8_t {
    EVENT_NONE = 0,
    EVENT_TAP = 1,
    EVENT_TOUCH = 2,
    EVENT_RFID_TAG = 3,
    EVENT_NETWORK_MESSAGE = 4,
    EVENT_TIMER = 5
};

enum NodeRole : uint8_t {
    ROLE_LIGHT = 0,
    ROLE_MASTER = 1
};

static inline uint8_t groupFromSwitch(uint8_t switchHigh) {
    return switchHigh ? GROUP_B : GROUP_A;
}

static inline uint8_t messageMatchesGroup(uint8_t localGroupId, uint8_t messageGroupId) {
    return (messageGroupId == GROUP_ALL) || (messageGroupId == localGroupId);
}
/**
 * @brief FSM application instance data
 * 
 */

typedef struct messageToBeSent {
    uint8_t messageType;
    uint8_t stateId;
    uint8_t groupId;
    uint8_t lightId;
    uint8_t sourceLightId;
    uint8_t targetLightId;
    uint8_t eventId;
    uint16_t color_r;
    uint16_t color_g;
    uint16_t color_b;
    uint8_t illuminationMode;
    uint8_t squeezed;
    uint8_t tap;
    uint8_t modeAB;
    uint8_t touched;
    uint32_t value;
} message_t;

typedef struct Ctx {
	uint8_t isMaster;
	uint8_t role;
	uint8_t lightId;
	uint8_t groupId;
    uint8_t stateId;
    uint8_t requestedStateId;
    uint8_t activeLightId;
    uint8_t registeredLights[MAX_LIGHTS];
    uint64_t timerValue;
    uint32_t lastRegisterMillis;
    uint32_t lastRfidPollMillis;
	float	initialPressure;  
    
	message_t light[MAX_LIGHTS];
} Ctx, * CtxPtr ;

#endif  // REACTION_ESP_H_
