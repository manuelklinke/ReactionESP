#include "states/StateRegistry.h"
#include "reaction_Esp.h"
#include "states/InitState.h"
#include "states/classic1ofn/Classic1OfNState.h"

cfsm_TransitionFunction stateEnterFunctionFor(uint8_t stateId, uint8_t role) {
    if (stateId == STATE_INIT) {
        return InitState_enter;
    }
    if (stateId == STATE_CLASSIC_1_OF_N) {
        if (role == ROLE_MASTER) {
            return Classic1OfNMaster_enter;
        }
        if (role == ROLE_LIGHT) {
            return Classic1OfNLight_enter;
        }
    }
    return nullptr;
}
