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
