#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "tasks/control_task.h"
#include "gimbal/gimbal.h"
#include "gimbal/gimbal_configuration.h"
#include "sensors/as5600.h"
#include "utils/time_utils.h"
#include "hardware_configuration.h"
#include "task_context.h"

void control_task(void *pvParameters) {
    task_context_t *ctx = (task_context_t *)pvParameters;
    gimbal_t *gimbal = ctx->gimbal;

    TickType_t last = xTaskGetTickCount();

    for (;;) {
        gpio_put(TEST_PIN, 1);

        // Pan
        uint16_t pan_raw_angle = as5600_get_raw_angle(&gimbal->engine.pan_encoder);
        gimbal->state.pan_position = (float)AS5600_RAW_TO_DEGREES(pan_raw_angle);

        if (gimbal->state.gimbal_mode == GIMBAL_MODE_ARMED) {
            float pan_pid_output = pid_update(&gimbal->engine.pan_position_controller, gimbal->state.pan_position_setpoint,
                (float)pan_raw_angle, gimbal->state.pan_lower_limit, gimbal->state.pan_upper_limit);
            // float pan_pwm_duty_cycle = pid_normalise_output(&gimbal->engine.pan_position_controller, -100.0f, 100.0f); // TODO: standardise -100.0f and 100.0f into constants
            uint8_t pan_direction = pan_pid_output > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
            gpio_put(PAN_MOTOR_DIR_PIN, pan_direction);
            pwm_set_gpio_level(PAN_PWM_PIN, (uint16_t)(abs(pan_pid_output)));

            // Also update gimbal state
            gimbal->state.pan_motor_pwm = (uint16_t)(abs(pan_pid_output));
            gimbal->state.pan_motor_dir = pan_direction;
        } else if (gimbal->state.gimbal_mode == GIMBAL_MODE_FREE) {
            pid_reset(&gimbal->engine.pan_position_controller);
            gpio_put(PAN_MOTOR_DIR_PIN, 0);     // Default dir
            pwm_set_gpio_level(PAN_PWM_PIN, 0); // Default pwm

            // Also reset gimbal state
            gimbal->state.pan_motor_pwm = 0;
            gimbal->state.pan_motor_dir = 0;
        }

        gpio_put(TEST_PIN, 0);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(SECS2MSECS(FREQ2PERIOD(CONTROLS_FREQ))));
    }
}
