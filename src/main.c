#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gimbal/gimbal.h"
#include "hardware_configuration.h"
#include "task_manager.h"

int main() {
    stdio_init_all();

    setup_gpio();

    // Setup gimbal instance
    gimbal_t gimbal = {0};
    setup_gimbal(&gimbal);

    // Setup RTOS task manager
    task_manager_init(&gimbal);

    vTaskStartScheduler();
    for (;;);

    return 0;
}
