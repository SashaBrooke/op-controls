#include "pico/stdlib.h"
#include "bsp/board.h"
#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gimbal/gimbal.h"
#include "hardware_configuration.h"
#include "task_manager.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;

    gpio_put(ERROR_PIN, 1);
    taskDISABLE_INTERRUPTS();
    for (;;) {
        tight_loop_contents();
    }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE *pulIdleTaskStackSize) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                          StackType_t **ppxIdleTaskStackBuffer,
                                          configSTACK_DEPTH_TYPE *pulIdleTaskStackSize,
                                          BaseType_t xPassiveIdleTaskIndex) {
    (void)xPassiveIdleTaskIndex;
    static StaticTask_t xPassiveIdleTaskTCB;
    static StackType_t uxPassiveIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer   = &xPassiveIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxPassiveIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

static gimbal_t gimbal = {0};

int main() {
    board_init();

    setup_gpio();

    tusb_init();

    // Setup gimbal instance
    setup_gimbal(&gimbal);

    // Setup RTOS task manager
    task_manager_init(&gimbal);

    vTaskStartScheduler();
    for (;;);

    return 0;
}
