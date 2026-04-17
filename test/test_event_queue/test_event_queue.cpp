#include <Arduino.h>
#include "app/AppEvents.h"

void setup() {
    Serial.begin(115200);

    app_event_queue_t queue;
    appEventQueueInit(&queue);

    bool ok = true;

    for (uint8_t i = 0; i < APP_EVENT_QUEUE_CAPACITY; i++) {
        app_event_t event = {};
        event.eventId = EVENT_TAP;
        event.sourceLightId = i;
        event.groupId = GROUP_A;
        ok = ok && appEventQueuePush(&queue, &event);
    }

    app_event_t overflowEvent = {};
    overflowEvent.eventId = EVENT_TOUCH;
    overflowEvent.sourceLightId = 99;
    overflowEvent.groupId = GROUP_B;
    ok = ok && !appEventQueuePush(&queue, &overflowEvent);

    for (uint8_t i = 0; i < APP_EVENT_QUEUE_CAPACITY; i++) {
        app_event_t popped = {};
        ok = ok && appEventQueuePop(&queue, &popped);
        ok = ok && popped.eventId == EVENT_TAP;
        ok = ok && popped.sourceLightId == i;
        ok = ok && popped.groupId == GROUP_A;
    }
    app_event_t emptyPop = {};
    ok = ok && !appEventQueuePop(&queue, &emptyPop);

    app_event_t wrapIn[5] = {};
    wrapIn[0].eventId = EVENT_TAP;
    wrapIn[0].sourceLightId = 10;
    wrapIn[1].eventId = EVENT_TAP;
    wrapIn[1].sourceLightId = 11;
    wrapIn[2].eventId = EVENT_TAP;
    wrapIn[2].sourceLightId = 12;
    wrapIn[3].eventId = EVENT_TAP;
    wrapIn[3].sourceLightId = 13;
    wrapIn[4].eventId = EVENT_TOUCH;
    wrapIn[4].sourceLightId = 14;
    for (uint8_t i = 0; i < 5; i++) {
        wrapIn[i].groupId = GROUP_B;
        ok = ok && appEventQueuePush(&queue, &wrapIn[i]);
    }

    app_event_t wrapPop = {};
    ok = ok && appEventQueuePop(&queue, &wrapPop);
    ok = ok && wrapPop.sourceLightId == 10;
    ok = ok && appEventQueuePop(&queue, &wrapPop);
    ok = ok && wrapPop.sourceLightId == 11;

    app_event_t wrapExtra[3] = {};
    wrapExtra[0].eventId = EVENT_RFID_TAG;
    wrapExtra[0].sourceLightId = 20;
    wrapExtra[1].eventId = EVENT_RFID_TAG;
    wrapExtra[1].sourceLightId = 21;
    wrapExtra[2].eventId = EVENT_RFID_TAG;
    wrapExtra[2].sourceLightId = 22;
    for (uint8_t i = 0; i < 3; i++) {
        wrapExtra[i].groupId = GROUP_B;
        ok = ok && appEventQueuePush(&queue, &wrapExtra[i]);
    }

    const uint8_t expectedSourceOrder[] = {12, 13, 14, 20, 21, 22};
    const uint8_t expectedEventOrder[] = {EVENT_TAP, EVENT_TAP, EVENT_TOUCH, EVENT_RFID_TAG, EVENT_RFID_TAG, EVENT_RFID_TAG};
    for (uint8_t i = 0; i < sizeof(expectedSourceOrder); i++) {
        app_event_t popped = {};
        ok = ok && appEventQueuePop(&queue, &popped);
        ok = ok && popped.sourceLightId == expectedSourceOrder[i];
        ok = ok && popped.eventId == expectedEventOrder[i];
    }
    ok = ok && !appEventQueuePop(&queue, &wrapPop);

    Serial.println(ok ? "event queue test passed" : "event queue test failed");
}

void loop() {
}
