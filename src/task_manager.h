/**
 * @file task_manager.h
 * @brief Centralised task creation and management
 */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "gimbal/gimbal.h"
#include "task_context.h"

// Task priorities
#define PRIORITY_CONTROLS       5
#define PRIORITY_RX_COMMS       4
#define PRIORITY_COMMS_PROCESS  3
#define PRIORITY_STREAM         2
#define PRIORITY_TX_COMMS       1

// Task stack sizes
#define STACK_SIZE_CONTROLS       512
#define STACK_SIZE_RX_COMMS       256
#define STACK_SIZE_COMMS_PROCESS  512
#define STACK_SIZE_STREAM         256
#define STACK_SIZE_TX_COMMS       256

// Queue lengths
#define RX_QUEUE_LEN            8
#define TX_QUEUE_LEN            8

void task_manager_init(gimbal_t *gimbal);

#endif // TASK_MANAGER_H
