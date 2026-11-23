/**
 * @file task_manager.h
 * @brief Centralised task creation and management
 */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "gimbal/gimbal.h"
#include "task_context.h"

// Core affinity masks
#define CORE_0_MASK  (1 << 0)
#define CORE_1_MASK  (1 << 1)

// Task priorities (higher number = higher priority)
#define PRIORITY_SYNC           7
#define PRIORITY_CONTROLS       6
#define PRIORITY_USB            5
#define PRIORITY_COMMS_RX       4
#define PRIORITY_COMMS_TX       3
#define PRIORITY_COMMS_PROCESS  2
#define PRIORITY_STREAM         1

// Task stack sizes
#define STACK_SIZE_SYNC           256
#define STACK_SIZE_CONTROLS       1024
#define STACK_SIZE_USB            1024
#define STACK_SIZE_COMMS_RX       1024
#define STACK_SIZE_COMMS_PROCESS  1536
#define STACK_SIZE_STREAM         1024
#define STACK_SIZE_COMMS_TX       1024

// Queue lengths
#define RX_QUEUE_LEN  8
#define TX_QUEUE_LEN  16

void task_manager_init(gimbal_t *gimbal);

#endif // TASK_MANAGER_H
