#include "task_manager.h"
#include "comms/packet.h"
#include "tasks/control_task.h"
#include "tasks/comm_rx_task.h"
#include "tasks/comm_process_task.h"
#include "tasks/comm_tx_task.h"
#include "tasks/stream_task.h"
#include "tasks/sync_task.h"

static QueueHandle_t rxQueue = NULL;
static QueueHandle_t txQueue = NULL;

static task_context_t ctx = {0};

void task_manager_init(gimbal_t *gimbal) {
    // Setup RX/TX queues
    rxQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(packet_t));
    txQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(packet_t));

    // Setup task context
    ctx.rxQueue = rxQueue;
    ctx.txQueue = txQueue;
    ctx.gimbal = gimbal;

    // Create tasks
    xTaskCreate(sync_task,         "sync",       STACK_SIZE_SYNC,          &ctx, PRIORITY_SYNC,          NULL);
    xTaskCreate(control_task,      "controls",   STACK_SIZE_CONTROLS,      &ctx, PRIORITY_CONTROLS,      NULL);
    // xTaskCreate(comm_rx_task,      "rx-comms",   STACK_SIZE_COMMS_RX,      &ctx, PRIORITY_COMMS_RX,      NULL);
    // xTaskCreate(comm_process_task, "comms-proc", STACK_SIZE_COMMS_PROCESS, &ctx, PRIORITY_COMMS_PROCESS, NULL);
    xTaskCreate(stream_task,       "stream",     STACK_SIZE_STREAM,        &ctx, PRIORITY_STREAM,        NULL);
    xTaskCreate(comm_tx_task,      "tx-comms",   STACK_SIZE_COMMS_TX,      &ctx, PRIORITY_COMMS_TX,      NULL);
}
