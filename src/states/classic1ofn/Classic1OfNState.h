#ifndef STATES_CLASSIC_1_OF_N_STATE_H
#define STATES_CLASSIC_1_OF_N_STATE_H

#include "stm.h"
#include "reaction_Esp.h"

#define CLASSIC_ACTIVE_RED 0
#define CLASSIC_ACTIVE_GREEN 255
#define CLASSIC_ACTIVE_BLUE 0
#define CLASSIC_OFF_RED 0
#define CLASSIC_OFF_GREEN 0
#define CLASSIC_OFF_BLUE 0

uint8_t classicChooseNextLight(uint8_t currentLightId, const uint8_t* registeredLights, uint8_t maxLights, uint32_t randomValue);
message_t classicMakeActiveLightMessage(uint8_t groupId, uint8_t lightId);
message_t classicMakeInactiveLightMessage(uint8_t groupId, uint8_t lightId);
void Classic1OfNMaster_enter(struct cfsm_Ctx* stateMachine);
void Classic1OfNLight_enter(struct cfsm_Ctx* stateMachine);

#endif
