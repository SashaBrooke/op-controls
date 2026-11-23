#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tusb.h"

#include "tasks/comm_rx_task.h"
#include "task_context.h"
#include "comms/packet.h"

static TaskHandle_t xCommRxTaskHandle = NULL;

// Called automatically by tud_task() whenever new bytes arrive on USB CDC
void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    if (xCommRxTaskHandle != NULL) {
        xTaskNotifyGive(xCommRxTaskHandle);
    }
}

void comm_rx_task(void *pvParameters) {
    task_context_t *ctx = (task_context_t *)pvParameters;
    xCommRxTaskHandle = xTaskGetCurrentTaskHandle();

    uint8_t rx_buf[MAX_WIRE_FRAME_SIZE];
    size_t rx_idx = 0;
    raw_pb_frame_t raw_pb;

    for (;;) {
        // Sleep at 0% CPU until tud_cdc_rx_cb() notifies us (50ms fallback timeout)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        // Drain all available bytes from TinyUSB CDC FIFO
        while (tud_cdc_available() > 0) {
            uint8_t byte_in;
            if (tud_cdc_read(&byte_in, 1) == 1) {
                if (byte_in == 0x00) {
                    // End-of-frame delimiter reached
                    if (rx_idx > 0) {
                        if (packet_unframe(rx_buf, rx_idx, &raw_pb)) {
                            xQueueSend(ctx->rxQueue, &raw_pb, pdMS_TO_TICKS(5));
                        }
                        rx_idx = 0;
                    }
                } else if (rx_idx < sizeof(rx_buf)) {
                    rx_buf[rx_idx++] = byte_in;
                } else {
                    // Buffer overflow protection: discard corrupted frame and wait for next 0x00
                    rx_idx = 0;
                }
            }
        }
    }
}
