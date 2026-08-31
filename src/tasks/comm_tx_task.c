#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "pico/stdlib.h"

#include "comm_tx_task.h"
#include "task_context.h"
#include "comms/packet.h"
#include "hardware_configuration.h"
#include "software_configuration.h"
#include "utils/time_utils.h"

void comm_tx_task(void *pvParameters) {
    task_context_t *ctx = (task_context_t *)pvParameters;

    packet_t packet;

    TickType_t last = xTaskGetTickCount();

    for (;;) {
        // Wait for packet from TX queue
        if (xQueueReceive(ctx->txQueue, &packet, 0) == pdTRUE) {
            // Only pulse when transmitting
            gpio_put(TEST_PIN, 1);

            // Write packet to USB stdio
            fwrite(packet.data, 1, packet.len, stdout);
            fflush(stdout);

            gpio_put(TEST_PIN, 0);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(SECS2MSECS(FREQ2PERIOD(CONTROLS_FREQ))));
    }
}
