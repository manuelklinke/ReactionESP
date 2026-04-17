#include "app/AppEvents.h"

void appEventQueueInit(app_event_queue_t* queue) {
    if (queue == nullptr) {
        return;
    }

    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

bool appEventQueuePush(app_event_queue_t* queue, const app_event_t* event) {
    if (queue == nullptr || event == nullptr) {
        return false;
    }

    noInterrupts();

    if (queue->count >= APP_EVENT_QUEUE_CAPACITY) {
        interrupts();
        return false;
    }

    queue->events[queue->tail] = *event;
    queue->tail = (queue->tail + 1) % APP_EVENT_QUEUE_CAPACITY;
    queue->count++;
    interrupts();
    return true;
}

bool appEventQueuePop(app_event_queue_t* queue, app_event_t* event) {
    if (queue == nullptr || event == nullptr) {
        return false;
    }

    noInterrupts();

    if (queue->count == 0) {
        interrupts();
        return false;
    }

    *event = queue->events[queue->head];
    queue->head = (queue->head + 1) % APP_EVENT_QUEUE_CAPACITY;
    queue->count--;
    interrupts();
    return true;
}
