/**
 * @brief  Blink CFSM example: OFF State Implementation
 *
 */

#include <Arduino.h>
//#include <Wire.h>
#include <stm.h>

#include "reaction_Esp.h"
//#include "oState.h"

/**
 * @brief Cyclic processing CFSM operaion for ON state
 * 
 * Transition to Off state if time in state is larger/equal 1 second. 
 * 
 * @param fsm CFSM state machine context
 */
static void InitState_process(cfsm_Ctx * fsm)
{
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;

    /*if ((millis() - ctx->turnOnTimeMillis) >= 1000ull)
    {
        Serial.println(F("OnState: LED On time has expired !"));
        cfsm_transition(fsm, OffState_enter);
    }*/
}

/**
 * @brief CFSM state leave operation for ON state
 * 
 * Only used to print leave operation to serial log in this example.
 * 
 * @param fsm CFSM state machine context
 */
static void InitState_leave( cfsm_Ctx * fsm)
{
    
}

/**
 * @brief CFSM enter operation for ON state
 * 
 * Record time and turn LED on.
 * 
 * @param fsm CFSM state machine context
 */
void OnState_enter(cfsm_Ctx * fsm)
{
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;

    

    //digitalWrite(ctx->ledPin, HIGH);  /* turn LED on            */
    ctx->timerValue = millis(); /* record time in context */

    /* register ON state hanlders with CFSM */
    fsm->onProcess = InitState_process;
    fsm->onLeave = InitState_leave;
}