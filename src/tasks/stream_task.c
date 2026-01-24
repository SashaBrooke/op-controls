#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"

#include "tasks/stream_task.h"
#include "gimbal/gimbal.h"
#include "task_context.h"
#include "hardware_configuration.h"

void stream_task(void *pvParameters) {
    task_context_t *ctx = (task_context_t *)pvParameters;
    gimbal_t *gimbal = ctx->gimbal;

    TickType_t last = xTaskGetTickCount();
    
    for (;;) {
        gpio_put(TEST_PIN, 1);
        
        const char *msg = "01234567890123456789012345678901234567890123456789012345678912\r\n";
        fwrite(msg, 1, 64, stdout);

        gpio_put(TEST_PIN, 0);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}
