#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "tasks/comm_process_task.h"
#include "task_context.h"
#include "comms/packet.h"
#include "comms/proto_codec.h"
#include "comms/command.h"

void comm_process_task(void *pvParameters) {
    task_context_t *ctx = (task_context_t *)pvParameters;

    raw_pb_frame_t raw_pb;
    opcomms_intcomm_Command cmd;
    opcomms_intcomm_Ack ack;
    wire_frame_t tx_wire;

    for (;;) {
        if (xQueueReceive(ctx->rxQueue, &raw_pb, portMAX_DELAY) == pdTRUE) {
            if (proto_decode_command(&raw_pb, &cmd)) {
                xSemaphoreTake(ctx->gimbalMutex, portMAX_DELAY);
                command_dispatch(&cmd, ctx->gimbal, &ack);
                xSemaphoreGive(ctx->gimbalMutex);

                if (proto_encode_ack(&ack, &tx_wire)) {
                    xQueueSend(ctx->txQueue, &tx_wire, pdMS_TO_TICKS(10));
                }
            }
        }
    }
}
