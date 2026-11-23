#include "pico/stdlib.h"
#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tasks/comm_tx_task.h"
#include "task_context.h"
#include "comms/packet.h"
#include "hardware_configuration.h"

void comm_tx_task(void *pvParameters) {
    task_context_t *ctx = (task_context_t *)pvParameters;
    wire_frame_t packet;

    for (;;) {
        if (xQueueReceive(ctx->txQueue, &packet, portMAX_DELAY) == pdTRUE) {
            if (tud_cdc_connected()) {
                gpio_put(TEST_PIN, 1);

                size_t sent = 0;
                while (sent < packet.len) {
                    uint32_t written = tud_cdc_write(packet.data + sent, packet.len - sent);
                    sent += written;
                    if (sent < packet.len) {
                        tud_cdc_write_flush();
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                }
                tud_cdc_write_flush();

                gpio_put(TEST_PIN, 0);
            }
        }
    }
}
