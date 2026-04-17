#ifndef SRC_STATES_STATEREGISTRY_H
#define SRC_STATES_STATEREGISTRY_H

#include "reaction_Esp.h"
#include "stm.h"

cfsm_TransitionFunction stateEnterFunctionFor(uint8_t stateId, uint8_t role);

#endif
