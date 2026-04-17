#include <Arduino.h>
#include <ESPNowW.h>
#include "stm.h"
#include "reaction_Esp.h"
#include "app/AppEvents.h"
#include "app/NodeConfig.h"
#include "app/Protocol.h"
#include "states/InitState.h"
#include "states/StateRegistry.h"

static void InitState_process(cfsm_Ctx* fsm) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    if (ctx->role == ROLE_LIGHT && (millis() - ctx->lastRegisterMillis) >= 1000UL) {
        ctx->lastRegisterMillis = millis();
        message_t message = makeRegisterMessage(ctx->lightId, ctx->groupId);
        esp_now_send(MasterAddress, (uint8_t*)&message, sizeof(message));
    }
    if (ctx->requestedStateId != STATE_INIT) {
        cfsm_TransitionFunction enterFunction = stateEnterFunctionFor(ctx->requestedStateId, ctx->role);
        if (enterFunction != nullptr) {
            ctx->stateId = ctx->requestedStateId;
            cfsm_transition(fsm, enterFunction);
        }
    }
}

static void InitState_event(cfsm_Ctx* fsm, int eventId) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    if (eventId == EVENT_NETWORK_MESSAGE && ctx->requestedStateId != STATE_INIT) {
        cfsm_TransitionFunction enterFunction = stateEnterFunctionFor(ctx->requestedStateId, ctx->role);
        if (enterFunction != nullptr) {
            ctx->stateId = ctx->requestedStateId;
            cfsm_transition(fsm, enterFunction);
        }
    }
}

static void InitState_leave(cfsm_Ctx* fsm) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    ctx->timerValue = millis();
}

void InitState_enter(cfsm_Ctx* fsm) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    ctx->stateId = STATE_INIT;
    ctx->requestedStateId = STATE_INIT;
    ctx->activeLightId = NO_ACTIVE_LIGHT;
    ctx->timerValue = millis();
    fsm->onProcess = InitState_process;
    fsm->onEvent = InitState_event;
    fsm->onLeave = InitState_leave;
}
