#include "states/classic1ofn/Classic1OfNState.h"

static void Classic1OfNLight_process(cfsm_Ctx* fsm) {
    (void)fsm;
}

static void Classic1OfNLight_event(cfsm_Ctx* fsm, int eventId) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    if (eventId != EVENT_NETWORK_MESSAGE) {
        return;
    }

    message_t message = ctx->light[ctx->lightId];
    if (message.messageType != MSG_LIGHT_STATE) {
        return;
    }

    if (message.targetLightId != ctx->lightId) {
        return;
    }

    ctx->light[ctx->lightId] = message;
}

void Classic1OfNLight_enter(cfsm_Ctx* stateMachine) {
    CtxPtr ctx = (CtxPtr)stateMachine->ctxPtr;
    ctx->stateId = STATE_CLASSIC_1_OF_N;
    stateMachine->onProcess = Classic1OfNLight_process;
    stateMachine->onEvent = Classic1OfNLight_event;
    stateMachine->onLeave = nullptr;
}
