#include "Arduino.h"
#include "states/InitState.h"

#define MAX_LIGHTS 8
#define PRESSURE_OFFSET 100
/**
 * @brief FSM application instance data
 * 
 */

typedef struct messageToBeSent {
    uint8_t lightId;
    uint16_t color_r;
    uint16_t color_g;
    uint16_t color_b;
    uint8_t illuminationMode;
    uint8_t squeezed;
    uint8_t tap;
    uint8_t modeAB;
    uint8_t touched;

} message_t;

typedef struct Ctx {
	uint8_t isMaster;
	uint8_t lightId;
    uint8_t stateId;
    uint64_t timerValue;
	float	initialPressure;  

	message_t light[MAX_LIGHTS];

} Ctx, * CtxPtr ;