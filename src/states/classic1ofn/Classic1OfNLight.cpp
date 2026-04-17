#include "states/classic1ofn/Classic1OfNState.h"

static void Classic1OfNLight_process(cfsm_Ctx* fsm) {
    (void)fsm;
}

static void Classic1OfNLight_event(cfsm_Ctx* fsm, int eventId) {
    (void)fsm;
    (void)eventId;
}

void Classic1OfNLight_enter(cfsm_Ctx* stateMachine) {
    CtxPtr ctx = (CtxPtr)stateMachine->ctxPtr;
    ctx->stateId = STATE_CLASSIC_1_OF_N;
    stateMachine->onProcess = Classic1OfNLight_process;
    stateMachine->onEvent = Classic1OfNLight_event;
    stateMachine->onLeave = nullptr;
}
