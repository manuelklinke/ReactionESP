#include "states/StateRegistry.h"
#include "reaction_Esp.h"
#include "states/InitState.h"

cfsm_TransitionFunction stateEnterFunctionFor(uint8_t stateId, uint8_t role) {
    (void)role;
    if (stateId == STATE_INIT) {
        return InitState_enter;
    }
    return nullptr;
}
