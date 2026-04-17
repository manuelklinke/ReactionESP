#include <Arduino.h>
#include "app/Protocol.h"
#include "states/classic1ofn/Classic1OfNState.h"

static void Classic1OfNMaster_process(cfsm_Ctx* fsm) {
    (void)fsm;
}

static void Classic1OfNMaster_event(cfsm_Ctx* fsm, int eventId) {
    (void)fsm;
    (void)eventId;
}

uint8_t classicChooseNextLight(uint8_t currentLightId, const uint8_t* registeredLights, uint8_t maxLights, uint32_t randomValue) {
    if (registeredLights == nullptr || maxLights <= 1) {
        return currentLightId;
    }

    uint8_t alternatives[MAX_LIGHTS];
    uint8_t alternativeCount = 0;
    uint8_t fallback = UNKNOWN_LIGHT_ID;

    for (uint8_t lightId = 1; lightId < maxLights && lightId < MAX_LIGHTS; lightId++) {
        if (!registeredLights[lightId]) {
            continue;
        }

        if (fallback == UNKNOWN_LIGHT_ID) {
            fallback = lightId;
        }

        if (lightId != currentLightId) {
            alternatives[alternativeCount++] = lightId;
        }
    }

    if (alternativeCount > 0) {
        return alternatives[randomValue % alternativeCount];
    }

    if (fallback != UNKNOWN_LIGHT_ID) {
        return fallback;
    }

    return currentLightId;
}

message_t classicMakeActiveLightMessage(uint8_t groupId, uint8_t lightId) {
    return makeLightStateMessage(STATE_CLASSIC_1_OF_N, groupId, lightId, CLASSIC_ACTIVE_RED, CLASSIC_ACTIVE_GREEN, CLASSIC_ACTIVE_BLUE);
}

message_t classicMakeInactiveLightMessage(uint8_t groupId, uint8_t lightId) {
    return makeLightStateMessage(STATE_CLASSIC_1_OF_N, groupId, lightId, CLASSIC_OFF_RED, CLASSIC_OFF_GREEN, CLASSIC_OFF_BLUE);
}

void Classic1OfNMaster_enter(cfsm_Ctx* stateMachine) {
    CtxPtr ctx = (CtxPtr)stateMachine->ctxPtr;
    ctx->stateId = STATE_CLASSIC_1_OF_N;
    stateMachine->onProcess = Classic1OfNMaster_process;
    stateMachine->onEvent = Classic1OfNMaster_event;
    stateMachine->onLeave = nullptr;
}
