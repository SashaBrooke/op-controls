#ifndef TASK_CONTEXT_H
#define TASK_CONTEXT_H

#include "FreeRTOS.h"
#include "queue.h"

typedef struct {
    QueueHandle_t rxQueue;
    QueueHandle_t txQueue;
    gimbal_t *gimbal;
} task_context_t;

#endif // TASK_CONTEXT_H
