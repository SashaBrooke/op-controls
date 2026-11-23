#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "task_manager.h"
#include "comms/packet.h"
#include "tasks/usb_task.h"
#include "tasks/control_task.h"
#include "tasks/comm_rx_task.h"
#include "tasks/comm_process_task.h"
#include "tasks/comm_tx_task.h"
#include "tasks/stream_task.h"
#include "tasks/sync_task.h"

static QueueHandle_t rxQueue = NULL;
static QueueHandle_t txQueue = NULL;
static SemaphoreHandle_t gimbalMutex = NULL;

static task_context_t ctx = {0};

void task_manager_init(gimbal_t *gimbal) {
    // Setup RX/TX queues
    rxQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(raw_pb_frame_t));
    txQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(wire_frame_t));

    gimbalMutex = xSemaphoreCreateMutex();

    // Setup task context
    ctx.rxQueue     = rxQueue;
    ctx.txQueue     = txQueue;
    ctx.gimbalMutex = gimbalMutex;
    ctx.gimbal      = gimbal;

    // Create tasks/handles
    TaskHandle_t hSyncTask    = NULL;
    TaskHandle_t hControlTask = NULL;
    TaskHandle_t hUsbTask     = NULL;
    TaskHandle_t hRxTask      = NULL;
    TaskHandle_t hProcessTask = NULL;
    TaskHandle_t hStreamTask  = NULL;
    TaskHandle_t hTxTask      = NULL;

    // Core 0 to handle real-time control loop
    xTaskCreate(sync_task,    "sync",     STACK_SIZE_SYNC,     &ctx, PRIORITY_SYNC,     &hSyncTask);
    xTaskCreate(control_task, "controls", STACK_SIZE_CONTROLS, &ctx, PRIORITY_CONTROLS, &hControlTask);

    vTaskCoreAffinitySet(hSyncTask,    CORE_0_MASK);
    vTaskCoreAffinitySet(hControlTask, CORE_0_MASK);

    // Core 1 to handle comms
    xTaskCreate(usb_task,          "usb",        STACK_SIZE_USB,           NULL, PRIORITY_USB,           &hUsbTask);
    xTaskCreate(comm_rx_task,      "rx-comms",   STACK_SIZE_COMMS_RX,      &ctx, PRIORITY_COMMS_RX,      &hRxTask);
    xTaskCreate(comm_process_task, "comms-proc", STACK_SIZE_COMMS_PROCESS, &ctx, PRIORITY_COMMS_PROCESS, &hProcessTask);
    xTaskCreate(stream_task,       "stream",     STACK_SIZE_STREAM,        &ctx, PRIORITY_STREAM,        &hStreamTask);
    xTaskCreate(comm_tx_task,      "tx-comms",   STACK_SIZE_COMMS_TX,      &ctx, PRIORITY_COMMS_TX,      &hTxTask);

    vTaskCoreAffinitySet(hUsbTask,     CORE_1_MASK);
    vTaskCoreAffinitySet(hRxTask,      CORE_1_MASK);
    vTaskCoreAffinitySet(hProcessTask, CORE_1_MASK);
    vTaskCoreAffinitySet(hStreamTask,  CORE_1_MASK);
    vTaskCoreAffinitySet(hTxTask,      CORE_1_MASK);
}
