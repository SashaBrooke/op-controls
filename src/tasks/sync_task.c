#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"

#include "sync_task.h"
#include "hardware_configuration.h"
#include "software_configuration.h"
#include "utils/time_utils.h"

void sync_task(void *pvParameters) {
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        // Very short pulse on test pin - used to sync oscilloscope
        gpio_put(TEST_PIN, 1);
        gpio_put(TEST_PIN, 0);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(SECS2MSECS(FREQ2PERIOD(CONTROLS_FREQ))));
    }
}
