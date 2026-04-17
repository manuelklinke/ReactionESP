# State Architecture and Classic 1 aus n Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the shared firmware foundation where every device starts in Init, registers with the master, receives a `stateId`, routes sensor events into active states, and runs the first complete game mode: "Klassiker: 1 aus n".

**Architecture:** Keep one firmware for all devices. Runtime role comes from Pin 16, group comes from Pin 14, and the master distributes the active state through ESP-NOW. Each game owns one shared header plus separate master and light state implementations.

**Tech Stack:** PlatformIO, Arduino framework for ESP8266, ESP-NOW via ESPNowW/native ESP-NOW API, ADXL345 tap sensor, PN532 RFID/NFC, NeoPixel, existing CFSM state machine in `src/stm.h` and `src/stm.c`.

---

## Scope

This plan implements the common state architecture and the first playable game, "Klassiker: 1 aus n". It does not implement Team-Duell, Runden-Timer, Capture the Flag, Memory-Sequenz, Trefferjagd, or Sensor-Mix. Those games should each get a separate plan after this foundation is stable.

The plan preserves the current static MAC address setup for the first implementation. Automatic pairing and dynamic Light-ID assignment are not part of this plan.

## Target File Structure

- Modify: `src/reaction_Esp.h`
  - Shared constants, roles, groups, state IDs, message types, event types, app context.
- Create: `src/app/AppEvents.h`
  - `app_event_t` type and a small fixed-size event queue for sensor/network events.
- Create: `src/app/AppEvents.cpp`
  - Event queue implementation.
- Create: `src/app/NodeConfig.h`
  - Pin constants, known master/slave MACs, helper functions for role, group, and peer lookup.
- Create: `src/app/NodeConfig.cpp`
  - Static MAC table and peer lookup implementation.
- Create: `src/app/Protocol.h`
  - Message construction helpers and message validation helpers.
- Create: `src/app/Protocol.cpp`
  - Implementation of protocol helpers.
- Modify: `src/states/InitState.h`
  - Correct Init-State declaration and event IDs used during registration.
- Replace: `src/states/InitState.c`
  - Rename to `src/states/InitState.cpp` in this plan so Arduino/C++ types are handled consistently.
- Create: `src/states/StateRegistry.h`
  - Map `stateId` and role to state enter functions.
- Create: `src/states/StateRegistry.cpp`
  - Registry implementation.
- Create: `src/states/classic1ofn/Classic1OfNState.h`
  - Shared constants and helpers for the first game.
- Create: `src/states/classic1ofn/Classic1OfNMaster.cpp`
  - Master state for random target selection and scoring.
- Create: `src/states/classic1ofn/Classic1OfNLight.cpp`
  - Light state for local LED display and tap event forwarding.
- Modify: `src/main.cpp`
  - Reduce to hardware setup, ESP-NOW setup, event production, state machine processing, and RFID polling hook.
- Create: `test/test_protocol_compile/test_protocol_compile.cpp`
  - Compile-time checks for protocol constants and helper behavior.
- Create: `test/test_event_queue/test_event_queue.cpp`
  - Compile-time and runtime smoke checks for event queue behavior.
- Create: `test/test_classic1ofn/test_classic1ofn.cpp`
  - Logic checks for next-target selection.
- Modify: `PROJEKTBESCHREIBUNG.md`
  - Mark the state architecture and "Klassiker: 1 aus n" as implementation target once complete.

---

### Task 1: Establish Shared Protocol Types

**Files:**
- Modify: `src/reaction_Esp.h`
- Create: `test/test_protocol_compile/test_protocol_compile.cpp`

- [ ] **Step 1: Write the failing compile test**

Create `test/test_protocol_compile/test_protocol_compile.cpp`:

```cpp
#include <Arduino.h>
#include "reaction_Esp.h"

static_assert(MAX_LIGHTS == 8, "MAX_LIGHTS must remain 8");
static_assert(GROUP_A == 0, "GROUP_A must be 0");
static_assert(GROUP_B == 1, "GROUP_B must be 1");
static_assert(GROUP_ALL == 255, "GROUP_ALL must be 255");
static_assert(STATE_INIT == 0, "Init state id must be 0");
static_assert(STATE_CLASSIC_1_OF_N == 1, "Classic state id must be 1");
static_assert(MSG_REGISTER == 1, "Register message type must be 1");
static_assert(MSG_STATE_SET == 3, "State-set message type must be 3");
static_assert(EVENT_TAP == 1, "Tap event id must be 1");
static_assert(sizeof(message_t) <= 64, "ESP-NOW message must stay compact");

void setup() {
    Serial.begin(115200);
    message_t msg = {};
    msg.messageType = MSG_SENSOR_EVENT;
    msg.stateId = STATE_CLASSIC_1_OF_N;
    msg.groupId = GROUP_A;
    msg.sourceLightId = 2;
    msg.targetLightId = 2;
    msg.eventId = EVENT_TAP;
    Serial.println(messageMatchesGroup(GROUP_A, msg.groupId) ? "protocol compile test passed" : "protocol compile test failed");
}

void loop() {
}
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
```

Expected: compile fails because `STATE_INIT`, `STATE_CLASSIC_1_OF_N`, `MSG_REGISTER`, `MSG_STATE_SET`, `EVENT_TAP`, and the new `message_t` fields do not exist yet.

- [ ] **Step 3: Implement the protocol types**

Update `src/reaction_Esp.h` so the top of the file contains these definitions before `message_t`:

```cpp
#include "Arduino.h"

#define MAX_LIGHTS 8
#define PRESSURE_OFFSET 100
#define GROUP_A 0
#define GROUP_B 1
#define GROUP_ALL 255
#define UNKNOWN_LIGHT_ID 255
#define NO_ACTIVE_LIGHT 255

enum StateId : uint8_t {
    STATE_INIT = 0,
    STATE_CLASSIC_1_OF_N = 1
};

enum MessageType : uint8_t {
    MSG_NONE = 0,
    MSG_REGISTER = 1,
    MSG_REGISTER_ACK = 2,
    MSG_STATE_SET = 3,
    MSG_SENSOR_EVENT = 4,
    MSG_LIGHT_STATE = 5,
    MSG_HEARTBEAT = 6
};

enum EventId : uint8_t {
    EVENT_NONE = 0,
    EVENT_TAP = 1,
    EVENT_TOUCH = 2,
    EVENT_RFID_TAG = 3,
    EVENT_NETWORK_MESSAGE = 4,
    EVENT_TIMER = 5
};

enum NodeRole : uint8_t {
    ROLE_LIGHT = 0,
    ROLE_MASTER = 1
};

static inline uint8_t groupFromSwitch(uint8_t switchHigh) {
    return switchHigh ? GROUP_B : GROUP_A;
}

static inline uint8_t messageMatchesGroup(uint8_t localGroupId, uint8_t messageGroupId) {
    return (messageGroupId == GROUP_ALL) || (messageGroupId == localGroupId);
}
```

Replace the existing `message_t` with:

```cpp
typedef struct messageToBeSent {
    uint8_t messageType;
    uint8_t stateId;
    uint8_t groupId;
    uint8_t sourceLightId;
    uint8_t targetLightId;
    uint8_t eventId;
    uint16_t color_r;
    uint16_t color_g;
    uint16_t color_b;
    uint8_t illuminationMode;
    uint8_t squeezed;
    uint8_t tap;
    uint8_t modeAB;
    uint8_t touched;
    uint32_t value;
} message_t;
```

Replace the existing `Ctx` with:

```cpp
typedef struct Ctx {
    uint8_t isMaster;
    uint8_t role;
    uint8_t lightId;
    uint8_t groupId;
    uint8_t stateId;
    uint8_t requestedStateId;
    uint8_t activeLightId;
    uint8_t registeredLights[MAX_LIGHTS];
    uint64_t timerValue;
    uint32_t lastRegisterMillis;
    uint32_t lastRfidPollMillis;
    float initialPressure;
    message_t light[MAX_LIGHTS];
} Ctx, * CtxPtr;
```

- [ ] **Step 4: Run the compile test and verify it passes**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
```

Expected: test compiles and serial output includes `protocol compile test passed`.

- [ ] **Step 5: Commit**

```bash
git add src/reaction_Esp.h test/test_protocol_compile/test_protocol_compile.cpp
git commit -m "feat: define shared reaction protocol"
```

---

### Task 2: Add Node Configuration and Peer Lookup

**Files:**
- Create: `src/app/NodeConfig.h`
- Create: `src/app/NodeConfig.cpp`
- Modify: `src/main.cpp`
- Test: `test/test_protocol_compile/test_protocol_compile.cpp`

- [ ] **Step 1: Extend the failing compile test**

Append this to `test/test_protocol_compile/test_protocol_compile.cpp` after the `message_t msg = {};` line:

```cpp
    uint8_t masterMac[6] = {0xCC, 0x7B, 0x5C, 0x4F, 0xD2, 0x84};
    uint8_t firstSlaveMac[6] = {0xD8, 0xBF, 0xC0, 0x17, 0xA6, 0x9A};
    bool lookupOk = (lightIdByMacAddress(masterMac) == 0) && (lightIdByMacAddress(firstSlaveMac) == 1);
    Serial.println(lookupOk ? "node lookup test passed" : "node lookup test failed");
```

Add this include at the top:

```cpp
#include "app/NodeConfig.h"
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
```

Expected: compile fails because `app/NodeConfig.h` and `lightIdByMacAddress` do not exist.

- [ ] **Step 3: Create node configuration header**

Create `src/app/NodeConfig.h`:

```cpp
#ifndef APP_NODE_CONFIG_H
#define APP_NODE_CONFIG_H

#include <Arduino.h>
#include "reaction_Esp.h"

#define PIN_NEOPIXEL 0
#define PIN_STATUS_LED 2
#define PIN_GROUP_SELECT 14
#define PIN_TOUCH 15
#define PIN_MASTER_SELECT 16
#define PIN_ADXL_INTERRUPT 13
#define PIN_PN532_INTERRUPT 12

#define ESPNOW_CHANNEL 1
#define STATIC_SLAVE_COUNT 3

extern uint8_t MasterAddress[6];
extern uint8_t SlaveAddress[STATIC_SLAVE_COUNT][6];

uint8_t roleFromMasterSwitch(uint8_t switchHigh);
uint8_t lightIdByMacAddress(const uint8_t* macAddr);
bool copyPeerAddressForLight(uint8_t lightId, uint8_t* destination);

#endif
```

- [ ] **Step 4: Create node configuration implementation**

Create `src/app/NodeConfig.cpp`:

```cpp
#include "app/NodeConfig.h"
#include <string.h>

uint8_t MasterAddress[6] = {0xCC, 0x7B, 0x5C, 0x4F, 0xD2, 0x84};

uint8_t SlaveAddress[STATIC_SLAVE_COUNT][6] = {
    {0xD8, 0xBF, 0xC0, 0x17, 0xA6, 0x9A},
    {0xCC, 0x7B, 0x5C, 0x50, 0x25, 0x1D},
    {0xCC, 0x7B, 0x5C, 0x4F, 0xDE, 0x3B}
};

uint8_t roleFromMasterSwitch(uint8_t switchHigh) {
    return switchHigh ? ROLE_MASTER : ROLE_LIGHT;
}

uint8_t lightIdByMacAddress(const uint8_t* macAddr) {
    if (memcmp(macAddr, MasterAddress, 6) == 0) {
        return 0;
    }

    for (uint8_t i = 0; i < STATIC_SLAVE_COUNT; i++) {
        if (memcmp(macAddr, SlaveAddress[i], 6) == 0) {
            return i + 1;
        }
    }

    return UNKNOWN_LIGHT_ID;
}

bool copyPeerAddressForLight(uint8_t lightId, uint8_t* destination) {
    if (destination == nullptr) {
        return false;
    }

    if (lightId == 0) {
        memcpy(destination, MasterAddress, 6);
        return true;
    }

    if (lightId >= 1 && lightId <= STATIC_SLAVE_COUNT) {
        memcpy(destination, SlaveAddress[lightId - 1], 6);
        return true;
    }

    return false;
}
```

- [ ] **Step 5: Remove duplicated MAC globals from `main.cpp`**

In `src/main.cpp`, remove the existing global `MasterAddress`, `broadcastAddress`, and `SlaveAddress` declarations. Add this include:

```cpp
#include "app/NodeConfig.h"
```

Keep `broadcastAddress` only if still used. If it is used, move it to `NodeConfig.cpp` and expose it in `NodeConfig.h` as:

```cpp
extern uint8_t BroadcastAddress[6];
```

- [ ] **Step 6: Run the test and firmware build**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
pio run -e d1_mini_pro
```

Expected: test passes and firmware builds without duplicate symbol errors.

- [ ] **Step 7: Commit**

```bash
git add src/app/NodeConfig.h src/app/NodeConfig.cpp src/main.cpp test/test_protocol_compile/test_protocol_compile.cpp
git commit -m "feat: add node configuration table"
```

---

### Task 3: Add Event Queue for Sensors and Network Messages

**Files:**
- Create: `src/app/AppEvents.h`
- Create: `src/app/AppEvents.cpp`
- Create: `test/test_event_queue/test_event_queue.cpp`

- [ ] **Step 1: Write the failing event queue test**

Create `test/test_event_queue/test_event_queue.cpp`:

```cpp
#include <Arduino.h>
#include "app/AppEvents.h"

void setup() {
    Serial.begin(115200);

    app_event_queue_t queue;
    appEventQueueInit(&queue);

    app_event_t event = {};
    event.eventId = EVENT_TAP;
    event.sourceLightId = 2;
    event.groupId = GROUP_A;

    bool pushOk = appEventQueuePush(&queue, &event);
    app_event_t popped = {};
    bool popOk = appEventQueuePop(&queue, &popped);
    bool emptyAfterPop = !appEventQueuePop(&queue, &popped);

    bool ok = pushOk && popOk && emptyAfterPop && popped.eventId == EVENT_TAP && popped.sourceLightId == 2;
    Serial.println(ok ? "event queue test passed" : "event queue test failed");
}

void loop() {
}
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
pio test -e d1_mini_pro -f test_event_queue
```

Expected: compile fails because `app/AppEvents.h` does not exist.

- [ ] **Step 3: Create event queue header**

Create `src/app/AppEvents.h`:

```cpp
#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include <Arduino.h>
#include "reaction_Esp.h"

#define APP_EVENT_QUEUE_CAPACITY 8

typedef struct app_event_t {
    uint8_t eventId;
    uint8_t stateId;
    uint8_t groupId;
    uint8_t sourceLightId;
    uint8_t targetLightId;
    uint32_t value;
    message_t message;
} app_event_t;

typedef struct app_event_queue_t {
    app_event_t events[APP_EVENT_QUEUE_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} app_event_queue_t;

void appEventQueueInit(app_event_queue_t* queue);
bool appEventQueuePush(app_event_queue_t* queue, const app_event_t* event);
bool appEventQueuePop(app_event_queue_t* queue, app_event_t* event);

#endif
```

- [ ] **Step 4: Create event queue implementation**

Create `src/app/AppEvents.cpp`:

```cpp
#include "app/AppEvents.h"

void appEventQueueInit(app_event_queue_t* queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

bool appEventQueuePush(app_event_queue_t* queue, const app_event_t* event) {
    if (queue->count >= APP_EVENT_QUEUE_CAPACITY) {
        return false;
    }

    queue->events[queue->tail] = *event;
    queue->tail = (queue->tail + 1) % APP_EVENT_QUEUE_CAPACITY;
    queue->count++;
    return true;
}

bool appEventQueuePop(app_event_queue_t* queue, app_event_t* event) {
    if (queue->count == 0) {
        return false;
    }

    *event = queue->events[queue->head];
    queue->head = (queue->head + 1) % APP_EVENT_QUEUE_CAPACITY;
    queue->count--;
    return true;
}
```

- [ ] **Step 5: Run the event queue test**

Run:

```bash
pio test -e d1_mini_pro -f test_event_queue
```

Expected: serial output includes `event queue test passed`.

- [ ] **Step 6: Commit**

```bash
git add src/app/AppEvents.h src/app/AppEvents.cpp test/test_event_queue/test_event_queue.cpp
git commit -m "feat: add app event queue"
```

---

### Task 4: Add Protocol Helper Functions

**Files:**
- Create: `src/app/Protocol.h`
- Create: `src/app/Protocol.cpp`
- Modify: `test/test_protocol_compile/test_protocol_compile.cpp`

- [ ] **Step 1: Extend protocol test**

Add this include:

```cpp
#include "app/Protocol.h"
```

Add this block inside `setup()`:

```cpp
    message_t registerMsg = makeRegisterMessage(3, GROUP_B);
    bool registerOk = registerMsg.messageType == MSG_REGISTER &&
                      registerMsg.sourceLightId == 3 &&
                      registerMsg.groupId == GROUP_B &&
                      isValidMessageForGroup(&registerMsg, GROUP_B);

    message_t stateMsg = makeStateSetMessage(STATE_CLASSIC_1_OF_N, GROUP_A);
    bool stateOk = stateMsg.messageType == MSG_STATE_SET &&
                   stateMsg.stateId == STATE_CLASSIC_1_OF_N &&
                   stateMsg.groupId == GROUP_A;

    Serial.println(registerOk && stateOk ? "protocol helper test passed" : "protocol helper test failed");
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
```

Expected: compile fails because `app/Protocol.h` and helper functions do not exist.

- [ ] **Step 3: Create protocol header**

Create `src/app/Protocol.h`:

```cpp
#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <Arduino.h>
#include "reaction_Esp.h"

message_t makeRegisterMessage(uint8_t sourceLightId, uint8_t groupId);
message_t makeRegisterAckMessage(uint8_t targetLightId, uint8_t groupId);
message_t makeStateSetMessage(uint8_t stateId, uint8_t groupId);
message_t makeSensorEventMessage(uint8_t stateId, uint8_t groupId, uint8_t sourceLightId, uint8_t eventId, uint32_t value);
message_t makeLightStateMessage(uint8_t stateId, uint8_t groupId, uint8_t targetLightId, uint16_t red, uint16_t green, uint16_t blue);
bool isExpectedMessageSize(uint8_t len);
bool isValidMessageForGroup(const message_t* message, uint8_t localGroupId);

#endif
```

- [ ] **Step 4: Create protocol implementation**

Create `src/app/Protocol.cpp`:

```cpp
#include "app/Protocol.h"

static message_t makeBaseMessage(uint8_t messageType, uint8_t stateId, uint8_t groupId) {
    message_t message = {};
    message.messageType = messageType;
    message.stateId = stateId;
    message.groupId = groupId;
    message.sourceLightId = UNKNOWN_LIGHT_ID;
    message.targetLightId = UNKNOWN_LIGHT_ID;
    message.eventId = EVENT_NONE;
    return message;
}

message_t makeRegisterMessage(uint8_t sourceLightId, uint8_t groupId) {
    message_t message = makeBaseMessage(MSG_REGISTER, STATE_INIT, groupId);
    message.sourceLightId = sourceLightId;
    return message;
}

message_t makeRegisterAckMessage(uint8_t targetLightId, uint8_t groupId) {
    message_t message = makeBaseMessage(MSG_REGISTER_ACK, STATE_INIT, groupId);
    message.targetLightId = targetLightId;
    return message;
}

message_t makeStateSetMessage(uint8_t stateId, uint8_t groupId) {
    return makeBaseMessage(MSG_STATE_SET, stateId, groupId);
}

message_t makeSensorEventMessage(uint8_t stateId, uint8_t groupId, uint8_t sourceLightId, uint8_t eventId, uint32_t value) {
    message_t message = makeBaseMessage(MSG_SENSOR_EVENT, stateId, groupId);
    message.sourceLightId = sourceLightId;
    message.eventId = eventId;
    message.value = value;
    return message;
}

message_t makeLightStateMessage(uint8_t stateId, uint8_t groupId, uint8_t targetLightId, uint16_t red, uint16_t green, uint16_t blue) {
    message_t message = makeBaseMessage(MSG_LIGHT_STATE, stateId, groupId);
    message.targetLightId = targetLightId;
    message.color_r = red;
    message.color_g = green;
    message.color_b = blue;
    return message;
}

bool isExpectedMessageSize(uint8_t len) {
    return len == sizeof(message_t);
}

bool isValidMessageForGroup(const message_t* message, uint8_t localGroupId) {
    return message != nullptr && messageMatchesGroup(localGroupId, message->groupId);
}
```

- [ ] **Step 5: Run the protocol test**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
```

Expected: serial output includes `protocol helper test passed`.

- [ ] **Step 6: Commit**

```bash
git add src/app/Protocol.h src/app/Protocol.cpp test/test_protocol_compile/test_protocol_compile.cpp
git commit -m "feat: add protocol helpers"
```

---

### Task 5: Fix and Activate Init State

**Files:**
- Modify: `src/states/InitState.h`
- Delete: `src/states/InitState.c`
- Create: `src/states/InitState.cpp`
- Modify: `src/main.cpp`

- [ ] **Step 1: Write a compile expectation**

Add this include to `test/test_protocol_compile/test_protocol_compile.cpp`:

```cpp
#include "states/InitState.h"
```

Add this compile assertion near the other `static_assert` lines:

```cpp
static_assert(STATE_INIT == 0, "Init state must be available before any game state");
```

- [ ] **Step 2: Run firmware build and verify current mismatch**

Run:

```bash
pio run -e d1_mini_pro
```

Expected before the fix: build either fails when InitState is linked or the state remains inactive because `InitState_enter` is declared but the implementation uses `OnState_enter`.

- [ ] **Step 3: Replace InitState header**

Update `src/states/InitState.h`:

```cpp
#ifndef SRC_STATES_INITSTATE_H
#define SRC_STATES_INITSTATE_H

#include "stm.h"

extern void InitState_enter(struct cfsm_Ctx* stateMachine);

#endif
```

- [ ] **Step 4: Replace InitState implementation**

Remove `src/states/InitState.c` and create `src/states/InitState.cpp`:

```cpp
#include <Arduino.h>
#include "stm.h"
#include "reaction_Esp.h"
#include "app/AppEvents.h"
#include "states/InitState.h"
#include "states/StateRegistry.h"

static void InitState_process(cfsm_Ctx* fsm) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
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
```

- [ ] **Step 5: Wire CFSM into main setup**

In `src/main.cpp`, uncomment or add:

```cpp
static cfsm_Ctx stateMachine;
```

At the end of `setup()`, after hardware and ESP-NOW initialization:

```cpp
cfsm_init(&stateMachine, &Data);
cfsm_transition(&stateMachine, InitState_enter);
```

In `loop()`, call:

```cpp
cfsm_process(&stateMachine);
```

- [ ] **Step 6: Run firmware build**

Run:

```bash
pio run -e d1_mini_pro
```

Expected: build succeeds and `InitState_enter` resolves.

- [ ] **Step 7: Commit**

```bash
git add src/states/InitState.h src/states/InitState.cpp src/main.cpp test/test_protocol_compile/test_protocol_compile.cpp
git rm src/states/InitState.c
git commit -m "feat: activate init state"
```

---

### Task 6: Add State Registry

**Files:**
- Create: `src/states/StateRegistry.h`
- Create: `src/states/StateRegistry.cpp`
- Create: `src/states/classic1ofn/Classic1OfNState.h`
- Create: `src/states/classic1ofn/Classic1OfNMaster.cpp`
- Create: `src/states/classic1ofn/Classic1OfNLight.cpp`

- [ ] **Step 1: Write registry compile expectation**

Add this include to `test/test_protocol_compile/test_protocol_compile.cpp`:

```cpp
#include "states/StateRegistry.h"
```

Add this block inside `setup()`:

```cpp
    bool registryOk = stateEnterFunctionFor(STATE_INIT, ROLE_MASTER) != nullptr &&
                      stateEnterFunctionFor(STATE_CLASSIC_1_OF_N, ROLE_MASTER) != nullptr &&
                      stateEnterFunctionFor(STATE_CLASSIC_1_OF_N, ROLE_LIGHT) != nullptr;
    Serial.println(registryOk ? "state registry test passed" : "state registry test failed");
```

- [ ] **Step 2: Run test and verify it fails**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
```

Expected: compile fails because `StateRegistry.h` does not exist.

- [ ] **Step 3: Create classic game shared header**

Create `src/states/classic1ofn/Classic1OfNState.h`:

```cpp
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
void Classic1OfNMaster_enter(struct cfsm_Ctx* stateMachine);
void Classic1OfNLight_enter(struct cfsm_Ctx* stateMachine);

#endif
```

- [ ] **Step 4: Create registry header**

Create `src/states/StateRegistry.h`:

```cpp
#ifndef STATES_STATE_REGISTRY_H
#define STATES_STATE_REGISTRY_H

#include "stm.h"
#include "reaction_Esp.h"

cfsm_TransitionFunction stateEnterFunctionFor(uint8_t stateId, uint8_t role);

#endif
```

- [ ] **Step 5: Create registry implementation**

Create `src/states/StateRegistry.cpp`:

```cpp
#include "states/StateRegistry.h"
#include "states/InitState.h"
#include "states/classic1ofn/Classic1OfNState.h"

cfsm_TransitionFunction stateEnterFunctionFor(uint8_t stateId, uint8_t role) {
    if (stateId == STATE_INIT) {
        return InitState_enter;
    }

    if (stateId == STATE_CLASSIC_1_OF_N && role == ROLE_MASTER) {
        return Classic1OfNMaster_enter;
    }

    if (stateId == STATE_CLASSIC_1_OF_N && role == ROLE_LIGHT) {
        return Classic1OfNLight_enter;
    }

    return nullptr;
}
```

- [ ] **Step 6: Create minimal classic master and light files**

Create `src/states/classic1ofn/Classic1OfNMaster.cpp`:

```cpp
#include <Arduino.h>
#include "states/classic1ofn/Classic1OfNState.h"

uint8_t classicChooseNextLight(uint8_t currentLightId, const uint8_t* registeredLights, uint8_t maxLights, uint32_t randomValue) {
    uint8_t candidates[MAX_LIGHTS] = {};
    uint8_t candidateCount = 0;

    for (uint8_t i = 0; i < maxLights; i++) {
        if (registeredLights[i] && i != currentLightId) {
            candidates[candidateCount++] = i;
        }
    }

    if (candidateCount == 0) {
        return currentLightId;
    }

    return candidates[randomValue % candidateCount];
}

static void Classic1OfNMaster_process(cfsm_Ctx* fsm) {
    (void)fsm;
}

static void Classic1OfNMaster_event(cfsm_Ctx* fsm, int eventId) {
    (void)fsm;
    (void)eventId;
}

void Classic1OfNMaster_enter(cfsm_Ctx* fsm) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    ctx->stateId = STATE_CLASSIC_1_OF_N;
    fsm->onProcess = Classic1OfNMaster_process;
    fsm->onEvent = Classic1OfNMaster_event;
}
```

Create `src/states/classic1ofn/Classic1OfNLight.cpp`:

```cpp
#include <Arduino.h>
#include "states/classic1ofn/Classic1OfNState.h"

static void Classic1OfNLight_process(cfsm_Ctx* fsm) {
    (void)fsm;
}

static void Classic1OfNLight_event(cfsm_Ctx* fsm, int eventId) {
    (void)fsm;
    (void)eventId;
}

void Classic1OfNLight_enter(cfsm_Ctx* fsm) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    ctx->stateId = STATE_CLASSIC_1_OF_N;
    fsm->onProcess = Classic1OfNLight_process;
    fsm->onEvent = Classic1OfNLight_event;
}
```

- [ ] **Step 7: Run registry test and build**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
pio run -e d1_mini_pro
```

Expected: registry test passes and firmware builds.

- [ ] **Step 8: Commit**

```bash
git add src/states/StateRegistry.h src/states/StateRegistry.cpp src/states/classic1ofn/Classic1OfNState.h src/states/classic1ofn/Classic1OfNMaster.cpp src/states/classic1ofn/Classic1OfNLight.cpp test/test_protocol_compile/test_protocol_compile.cpp
git commit -m "feat: add state registry and classic state shell"
```

---

### Task 7: Implement Init Registration Flow

**Files:**
- Modify: `src/main.cpp`
- Modify: `src/states/InitState.cpp`
- Modify: `src/app/Protocol.cpp`

- [ ] **Step 1: Add registration behavior expectation**

In `test/test_protocol_compile/test_protocol_compile.cpp`, add:

```cpp
    message_t ack = makeRegisterAckMessage(2, GROUP_A);
    bool ackOk = ack.messageType == MSG_REGISTER_ACK && ack.targetLightId == 2 && ack.groupId == GROUP_A;
    Serial.println(ackOk ? "register ack test passed" : "register ack test failed");
```

- [ ] **Step 2: Run test and confirm current protocol helpers support it**

Run:

```bash
pio test -e d1_mini_pro -f test_protocol_compile
```

Expected: test passes. If it fails, fix only `makeRegisterAckMessage`.

- [ ] **Step 3: Update ESP-NOW receive callback**

In `src/main.cpp`, change receive handling to:

```cpp
void messageReceived(uint8_t* macAddr, uint8_t* incomingData, uint8_t len) {
    if (!isExpectedMessageSize(len)) {
        Serial.print("Unexpected message size: ");
        Serial.println(len);
        return;
    }

    memcpy(&ReceivedMessage, incomingData, sizeof(ReceivedMessage));
    if (!isValidMessageForGroup(&ReceivedMessage, Data.groupId)) {
        Serial.println("Message ignored, different group");
        return;
    }

    app_event_t event = {};
    event.eventId = EVENT_NETWORK_MESSAGE;
    event.stateId = ReceivedMessage.stateId;
    event.groupId = ReceivedMessage.groupId;
    event.sourceLightId = ReceivedMessage.sourceLightId;
    event.targetLightId = ReceivedMessage.targetLightId;
    event.message = ReceivedMessage;
    appEventQueuePush(&EventQueue, &event);
}
```

Add these globals near `Data`:

```cpp
static app_event_queue_t EventQueue;
```

Add these includes:

```cpp
#include "app/AppEvents.h"
#include "app/Protocol.h"
```

- [ ] **Step 4: Initialize event queue and role fields**

In `setup()`, after reading pins:

```cpp
Data.role = roleFromMasterSwitch(digitalRead(PIN_MASTER_SELECT));
Data.isMaster = Data.role == ROLE_MASTER;
Data.groupId = groupFromSwitch(digitalRead(PIN_GROUP_SELECT));
Data.requestedStateId = STATE_INIT;
Data.stateId = STATE_INIT;
Data.activeLightId = NO_ACTIVE_LIGHT;
Data.registeredLights[Data.lightId] = 1;
appEventQueueInit(&EventQueue);
```

- [ ] **Step 5: Dispatch queued events in loop**

In `loop()`, after sensor polling and before `cfsm_process(&stateMachine)`:

```cpp
app_event_t event = {};
while (appEventQueuePop(&EventQueue, &event)) {
    if (event.eventId == EVENT_NETWORK_MESSAGE && event.message.messageType == MSG_STATE_SET) {
        Data.requestedStateId = event.message.stateId;
    }
    cfsm_event(&stateMachine, event.eventId);
}
```

- [ ] **Step 6: Add slave registration send from InitState**

In `src/states/InitState.cpp`, inside `InitState_process`, before checking `requestedStateId`:

```cpp
    if (ctx->role == ROLE_LIGHT && (millis() - ctx->lastRegisterMillis) >= 1000UL) {
        ctx->lastRegisterMillis = millis();
        message_t message = makeRegisterMessage(ctx->lightId, ctx->groupId);
        esp_now_send(MasterAddress, (uint8_t*)&message, sizeof(message));
    }
```

Add includes:

```cpp
#include <ESPNowW.h>
#include "app/NodeConfig.h"
#include "app/Protocol.h"
```

- [ ] **Step 7: Add master handling for register messages**

In `loop()`, when popping network events, handle register messages before calling `cfsm_event`:

```cpp
if (Data.role == ROLE_MASTER && event.message.messageType == MSG_REGISTER) {
    uint8_t lightId = event.message.sourceLightId;
    if (lightId < MAX_LIGHTS) {
        Data.registeredLights[lightId] = 1;
        message_t ack = makeRegisterAckMessage(lightId, Data.groupId);
        uint8_t peerAddress[6] = {};
        if (copyPeerAddressForLight(lightId, peerAddress)) {
            esp_now_send(peerAddress, (uint8_t*)&ack, sizeof(ack));
        }
    }
}
```

- [ ] **Step 8: Run build and do serial smoke test**

Run:

```bash
pio run -e d1_mini_pro
```

Then upload to one master and one light. Expected serial behavior:

- Master prints its role and group.
- Light sends `MSG_REGISTER` roughly once per second while in Init.
- Master records the light in `registeredLights`.
- No messages from the other group are accepted.

- [ ] **Step 9: Commit**

```bash
git add src/main.cpp src/states/InitState.cpp src/app/Protocol.cpp
git commit -m "feat: add init registration flow"
```

---

### Task 8: Implement Classic 1 aus n Target Selection

**Files:**
- Modify: `src/states/classic1ofn/Classic1OfNState.h`
- Modify: `src/states/classic1ofn/Classic1OfNMaster.cpp`
- Create: `test/test_classic1ofn/test_classic1ofn.cpp`

- [ ] **Step 1: Write target selection test**

Create `test/test_classic1ofn/test_classic1ofn.cpp`:

```cpp
#include <Arduino.h>
#include "states/classic1ofn/Classic1OfNState.h"

void setup() {
    Serial.begin(115200);

    uint8_t registered[MAX_LIGHTS] = {};
    registered[1] = 1;
    registered[2] = 1;
    registered[3] = 1;

    uint8_t nextFrom1 = classicChooseNextLight(1, registered, MAX_LIGHTS, 0);
    uint8_t nextFrom2 = classicChooseNextLight(2, registered, MAX_LIGHTS, 1);
    uint8_t nextFrom3 = classicChooseNextLight(3, registered, MAX_LIGHTS, 2);

    bool ok = nextFrom1 != 1 &&
              nextFrom2 != 2 &&
              nextFrom3 != 3 &&
              nextFrom1 < MAX_LIGHTS &&
              nextFrom2 < MAX_LIGHTS &&
              nextFrom3 < MAX_LIGHTS;

    Serial.println(ok ? "classic target test passed" : "classic target test failed");
}

void loop() {
}
```

- [ ] **Step 2: Run the test**

Run:

```bash
pio test -e d1_mini_pro -f test_classic1ofn
```

Expected: passes if Task 6 implementation is correct. If it fails, fix only `classicChooseNextLight`.

- [ ] **Step 3: Add helper for active light message**

Add to `src/states/classic1ofn/Classic1OfNState.h`:

```cpp
message_t classicMakeActiveLightMessage(uint8_t groupId, uint8_t lightId);
message_t classicMakeInactiveLightMessage(uint8_t groupId, uint8_t lightId);
```

Add to `src/states/classic1ofn/Classic1OfNMaster.cpp`:

```cpp
message_t classicMakeActiveLightMessage(uint8_t groupId, uint8_t lightId) {
    return makeLightStateMessage(STATE_CLASSIC_1_OF_N, groupId, lightId, CLASSIC_ACTIVE_RED, CLASSIC_ACTIVE_GREEN, CLASSIC_ACTIVE_BLUE);
}

message_t classicMakeInactiveLightMessage(uint8_t groupId, uint8_t lightId) {
    return makeLightStateMessage(STATE_CLASSIC_1_OF_N, groupId, lightId, CLASSIC_OFF_RED, CLASSIC_OFF_GREEN, CLASSIC_OFF_BLUE);
}
```

Add include:

```cpp
#include "app/Protocol.h"
```

- [ ] **Step 4: Run classic test and firmware build**

Run:

```bash
pio test -e d1_mini_pro -f test_classic1ofn
pio run -e d1_mini_pro
```

Expected: test passes and firmware builds.

- [ ] **Step 5: Commit**

```bash
git add src/states/classic1ofn/Classic1OfNState.h src/states/classic1ofn/Classic1OfNMaster.cpp test/test_classic1ofn/test_classic1ofn.cpp
git commit -m "feat: add classic target selection"
```

---

### Task 9: Implement Classic Master State

**Files:**
- Modify: `src/states/classic1ofn/Classic1OfNMaster.cpp`
- Modify: `src/main.cpp`

- [ ] **Step 1: Add active-state entry behavior**

Replace `Classic1OfNMaster_enter` with:

```cpp
void Classic1OfNMaster_enter(cfsm_Ctx* fsm) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    ctx->stateId = STATE_CLASSIC_1_OF_N;

    if (ctx->activeLightId == NO_ACTIVE_LIGHT) {
        ctx->activeLightId = classicChooseNextLight(NO_ACTIVE_LIGHT, ctx->registeredLights, MAX_LIGHTS, millis());
    }

    fsm->onProcess = Classic1OfNMaster_process;
    fsm->onEvent = Classic1OfNMaster_event;
}
```

- [ ] **Step 2: Add helper to send light state to one peer**

Add this static function to `Classic1OfNMaster.cpp`:

```cpp
static void sendLightStateToPeer(uint8_t lightId, const message_t* message) {
    uint8_t peerAddress[6] = {};
    if (copyPeerAddressForLight(lightId, peerAddress)) {
        esp_now_send(peerAddress, (uint8_t*)message, sizeof(*message));
    }
}
```

Add includes:

```cpp
#include <ESPNowW.h>
#include "app/NodeConfig.h"
```

- [ ] **Step 3: Broadcast current active light from process**

Replace `Classic1OfNMaster_process` with:

```cpp
static void Classic1OfNMaster_process(cfsm_Ctx* fsm) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;

    for (uint8_t lightId = 1; lightId < MAX_LIGHTS; lightId++) {
        if (!ctx->registeredLights[lightId]) {
            continue;
        }

        message_t message = lightId == ctx->activeLightId
            ? classicMakeActiveLightMessage(ctx->groupId, lightId)
            : classicMakeInactiveLightMessage(ctx->groupId, lightId);

        sendLightStateToPeer(lightId, &message);
    }
}
```

- [ ] **Step 4: Handle tap events in master state**

Replace `Classic1OfNMaster_event` with:

```cpp
static void Classic1OfNMaster_event(cfsm_Ctx* fsm, int eventId) {
    CtxPtr ctx = (CtxPtr)fsm->ctxPtr;
    if (eventId != EVENT_NETWORK_MESSAGE) {
        return;
    }

    message_t eventMessage = ctx->light[0];
    if (eventMessage.messageType != MSG_SENSOR_EVENT || eventMessage.eventId != EVENT_TAP) {
        return;
    }

    if (eventMessage.sourceLightId != ctx->activeLightId) {
        return;
    }

    ctx->activeLightId = classicChooseNextLight(ctx->activeLightId, ctx->registeredLights, MAX_LIGHTS, millis());
}
```

- [ ] **Step 5: Store current network event for state handling**

In `src/main.cpp`, when popping an event before `cfsm_event`, add:

```cpp
if (event.eventId == EVENT_NETWORK_MESSAGE) {
    Data.light[0] = event.message;
}
```

- [ ] **Step 6: Run build**

Run:

```bash
pio run -e d1_mini_pro
```

Expected: firmware builds.

- [ ] **Step 7: Commit**

```bash
git add src/states/classic1ofn/Classic1OfNMaster.cpp src/main.cpp
git commit -m "feat: implement classic master state"
```

---

### Task 10: Implement Classic Light State

**Files:**
- Modify: `src/states/classic1ofn/Classic1OfNLight.cpp`
- Modify: `src/main.cpp`

- [ ] **Step 1: Apply received light state**

Replace `Classic1OfNLight_event` with:

```cpp
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
```

- [ ] **Step 2: Store light-state messages in main event dispatch**

In `src/main.cpp`, when popping a network event:

```cpp
if (event.message.messageType == MSG_LIGHT_STATE && event.message.targetLightId < MAX_LIGHTS) {
    Data.light[event.message.targetLightId] = event.message;
}
```

- [ ] **Step 3: Forward local tap events**

In `src/main.cpp`, when ADXL tap is detected on a light device:

```cpp
message_t tapMessage = makeSensorEventMessage(Data.stateId, Data.groupId, Data.lightId, EVENT_TAP, millis());
esp_now_send(MasterAddress, (uint8_t*)&tapMessage, sizeof(tapMessage));
```

Remove older direct writes to `MessageToBeSent[0].tap` for the same behavior.

- [ ] **Step 4: Drive NeoPixels from current light state**

In `loop()`, replace direct color logic with:

```cpp
message_t localLight = Data.light[Data.lightId];
for (uint8_t i = 0; i < 8; i++) {
    strip.setPixelColor(i, localLight.color_r, localLight.color_g, localLight.color_b);
}
strip.show();
```

- [ ] **Step 5: Run build and two-device smoke test**

Run:

```bash
pio run -e d1_mini_pro
```

Upload one master and one light. Expected behavior:

- Light registers with master.
- Master enters classic state after receiving a state-set trigger from Task 11.
- Active light shows green.
- Tap on active light sends `MSG_SENSOR_EVENT`.
- Master chooses a different active light when at least two lights are registered.

- [ ] **Step 6: Commit**

```bash
git add src/states/classic1ofn/Classic1OfNLight.cpp src/main.cpp
git commit -m "feat: implement classic light state"
```

---

### Task 11: Add RFID Polling Hook and State Selection

**Files:**
- Modify: `src/main.cpp`
- Create: `src/app/RfidTask.h`
- Create: `src/app/RfidTask.cpp`

- [ ] **Step 1: Create RFID task header**

Create `src/app/RfidTask.h`:

```cpp
#ifndef APP_RFID_TASK_H
#define APP_RFID_TASK_H

#include <Arduino.h>
#include <Adafruit_PN532.h>
#include "reaction_Esp.h"

uint8_t stateIdForRfidUid(const uint8_t* uid, uint8_t uidLength);
bool pollRfidForState(Adafruit_PN532* nfc, uint8_t* stateId);

#endif
```

- [ ] **Step 2: Create RFID task implementation**

Create `src/app/RfidTask.cpp`:

```cpp
#include "app/RfidTask.h"

uint8_t stateIdForRfidUid(const uint8_t* uid, uint8_t uidLength) {
    if (uid == nullptr || uidLength == 0) {
        return STATE_INIT;
    }

    return STATE_CLASSIC_1_OF_N;
}

bool pollRfidForState(Adafruit_PN532* nfc, uint8_t* stateId) {
    if (nfc == nullptr || stateId == nullptr) {
        return false;
    }

    uint8_t uid[7] = {};
    uint8_t uidLength = 0;
    bool success = nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    if (!success) {
        return false;
    }

    *stateId = stateIdForRfidUid(uid, uidLength);
    return true;
}
```

- [ ] **Step 3: Call RFID task only on master**

In `src/main.cpp`, add include:

```cpp
#include "app/RfidTask.h"
```

In `loop()`, add this guarded block:

```cpp
if (Data.role == ROLE_MASTER && (millis() - Data.lastRfidPollMillis) >= 250UL) {
    Data.lastRfidPollMillis = millis();
    uint8_t selectedStateId = STATE_INIT;
    if (pollRfidForState(&nfc, &selectedStateId) && selectedStateId != STATE_INIT) {
        Data.requestedStateId = selectedStateId;
        message_t stateMessage = makeStateSetMessage(selectedStateId, Data.groupId);
        for (uint8_t lightId = 1; lightId <= STATIC_SLAVE_COUNT; lightId++) {
            uint8_t peerAddress[6] = {};
            if (copyPeerAddressForLight(lightId, peerAddress)) {
                esp_now_send(peerAddress, (uint8_t*)&stateMessage, sizeof(stateMessage));
            }
        }
    }
}
```

- [ ] **Step 4: Document blocking behavior in code comment**

Immediately above the `pollRfidForState` call, add:

```cpp
// PN532 readPassiveTargetID can block until a card is read on this hardware setup.
// Keep RFID polling isolated here so game states do not contain blocking reader logic.
```

- [ ] **Step 5: Run build**

Run:

```bash
pio run -e d1_mini_pro
```

Expected: firmware builds.

- [ ] **Step 6: Hardware smoke test**

Upload to master and light. Present any RFID/NFC card to the master reader. Expected:

- Master sets `requestedStateId` to `STATE_CLASSIC_1_OF_N`.
- Master sends `MSG_STATE_SET`.
- Light receives `MSG_STATE_SET` and transitions out of Init.
- Serial logs show state change.

- [ ] **Step 7: Commit**

```bash
git add src/app/RfidTask.h src/app/RfidTask.cpp src/main.cpp
git commit -m "feat: select state from rfid"
```

---

### Task 12: Clean Up Main Loop Responsibilities

**Files:**
- Modify: `src/main.cpp`

- [ ] **Step 1: Remove obsolete direct game logic**

In `src/main.cpp`, remove blocks that hard-code:

```cpp
if(Data.light[0].tap == 1) { ... }
if(Data.light[2].tap == 1) { ... }
if(Data.light[3].tap == 1) { ... }
```

Remove the old master loop that sends `MessageToBeSent[i]` based on `Data.light[i+1]`.

- [ ] **Step 2: Keep main loop responsibilities limited**

After cleanup, `loop()` should perform only these high-level actions:

```cpp
Data.groupId = groupFromSwitch(digitalRead(PIN_GROUP_SELECT));
readLocalSensorsIntoEvents();
pollMasterRfidIfNeeded();
dispatchQueuedEventsToStateMachine();
cfsm_process(&stateMachine);
renderLocalOutputs();
delay(10);
```

If helper functions do not exist yet, create `static` functions inside `main.cpp` with these names and move existing code into them:

```cpp
static void readLocalSensorsIntoEvents();
static void pollMasterRfidIfNeeded();
static void dispatchQueuedEventsToStateMachine();
static void renderLocalOutputs();
```

- [ ] **Step 3: Run build**

Run:

```bash
pio run -e d1_mini_pro
```

Expected: firmware builds and `loop()` no longer contains game-specific color decisions.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "refactor: isolate main loop responsibilities"
```

---

### Task 13: Update Documentation

**Files:**
- Modify: `PROJEKTBESCHREIBUNG.md`

- [ ] **Step 1: Update implementation status**

In `PROJEKTBESCHREIBUNG.md`, change the "Umgesetzte Grundlagen" list to include:

```markdown
- aktive State-Machine mit Init-State
- Anmeldung der Reaktionslichter beim Master
- Verteilung der aktiven `stateId` durch den Master
- erster Spiel-State "Klassiker: 1 aus n"
```

- [ ] **Step 2: Update Klassiker status**

Change:

```markdown
Status: geplant als erster vollstaendiger Spielmodus.
```

to:

```markdown
Status: erste umgesetzte Spiellogik.
```

- [ ] **Step 3: Run markdown sanity check**

Run:

```bash
Select-String -Path PROJEKTBESCHREIBUNG.md -Pattern 'State-Architektur|Klassiker|stateId|Init-State'
```

Expected: output includes each searched term at least once.

- [ ] **Step 4: Commit**

```bash
git add PROJEKTBESCHREIBUNG.md
git commit -m "docs: update state architecture status"
```

---

### Task 14: Final Verification

**Files:**
- No source edits unless a verification failure identifies a specific fix.

- [ ] **Step 1: Run all PlatformIO tests**

Run:

```bash
pio test -e d1_mini_pro
```

Expected: all test folders build and run. Serial output includes:

- `protocol compile test passed`
- `node lookup test passed`
- `protocol helper test passed`
- `state registry test passed`
- `event queue test passed`
- `classic target test passed`

- [ ] **Step 2: Run firmware build**

Run:

```bash
pio run -e d1_mini_pro
```

Expected: build succeeds with no compile errors.

- [ ] **Step 3: Run two-device hardware smoke test**

Hardware setup:

- Device 1: Pin 16 active, Pin 14 set to Group A, PN532 connected.
- Device 2: Pin 16 inactive, Pin 14 set to Group A, ADXL345 connected.

Expected:

- Both devices boot into Init.
- Light registers with master.
- Master accepts the light registration.
- RFID card on master selects `STATE_CLASSIC_1_OF_N`.
- Master sends `MSG_STATE_SET`.
- Light enters classic light state.
- One active light displays green.
- Tap on active light is sent as `MSG_SENSOR_EVENT`.
- Master accepts tap only when `sourceLightId == activeLightId`.

- [ ] **Step 4: Run group isolation smoke test**

Hardware setup:

- Master in Group A.
- Light in Group B.

Expected:

- Light can send registration attempts.
- Master ignores messages with Group B.
- Master does not send Group A state messages to Group B devices.

- [ ] **Step 5: Check git status**

Run:

```bash
git status --short
```

Expected: clean working tree after all commits.

---

## Self-Review

Spec coverage:

- Same firmware on all devices: covered by Tasks 1, 2, 5, 6, 7, and 12.
- Init state: covered by Task 5.
- Registration with master: covered by Task 7.
- Waiting for `stateId`: covered by Tasks 5 and 7.
- Game as states: covered by Tasks 6, 8, 9, and 10.
- Master state and light state sharing a header: covered by Task 6.
- Sensor events handed to active state: covered by Tasks 3, 7, 9, and 10.
- RFID reader polled cyclically and isolated because it can block: covered by Task 11.
- Klassiker 1 aus n: covered by Tasks 8, 9, and 10.

Known implementation risks:

- ESP-NOW callback code should stay short. This plan queues network messages and lets `loop()` dispatch them.
- PN532 blocking behavior can still affect the master loop because ESP8266 has limited task isolation. This plan isolates the blocking call in one function so it can later be moved behind an interrupt or timeout-capable reader strategy.
- Static MAC addresses remain in use. Dynamic pairing is a separate feature.
