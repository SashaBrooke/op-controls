#include <stdint.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "tasks/stream_task.h"
#include "gimbal/gimbal.h"
#include "task_context.h"
#include "hardware_configuration.h"
#include "software_configuration.h"
#include "comms/packet.h"
#include "comms/proto_codec.h"
#include "utils/time_utils.h"

void stream_task(void *pvParameters) {
    task_context_t *ctx = (task_context_t *)pvParameters;
    gimbal_t *gimbal = ctx->gimbal;

    TickType_t last = xTaskGetTickCount();
    wire_frame_t tx_wire;

    for (;;) {
        uint16_t task_rate = 20; // Idle check rate (20 Hz) when streaming is disabled

        xSemaphoreTake(ctx->gimbalMutex, portMAX_DELAY);
        bool streaming       = gimbal->state.streaming;
        uint16_t stream_rate = gimbal->state.stream_rate;
        bool pan_pos_en      = gimbal->state.pan_pos_stream;
        bool pan_pid_en      = gimbal->state.pan_pid_stream;

        float pan_pos        = gimbal->state.pan_position;
        float pan_setpoint   = gimbal->state.pan_position_setpoint;
        uint8_t motor_dir    = gimbal->state.pan_motor_dir;
        uint32_t motor_pwm   = gimbal->state.pan_motor_pwm;

        float p_term         = gimbal->engine.pan_position_controller.proportional;
        float i_term         = gimbal->engine.pan_position_controller.integrator;
        float d_term         = gimbal->engine.pan_position_controller.differentiator;
        float pid_out        = gimbal->engine.pan_position_controller.output;
        xSemaphoreGive(ctx->gimbalMutex);

        if (streaming && stream_rate > 0) {
            gpio_put(TEST_PIN, 1);
            task_rate = stream_rate;

            // Axis info stream
            if (pan_pos_en) {
                opcomms_intcomm_MotorDirection dir_enum = (motor_dir == 0)
                    ? opcomms_intcomm_MotorDirection_MOTOR_DIR_CW
                    : opcomms_intcomm_MotorDirection_MOTOR_DIR_CCW;

                if (proto_encode_axis_stream(opcomms_intcomm_Axis_AXIS_PAN,
                                             pan_pos, dir_enum, motor_pwm,
                                             &tx_wire)) {
                    xQueueSend(ctx->txQueue, &tx_wire, 0);
                }
            }

            // PID debug stream
            if (pan_pid_en) {
                if (proto_encode_pid_stream(opcomms_intcomm_Axis_AXIS_PAN,
                                            pan_setpoint, pan_pos,
                                            p_term, i_term, d_term, pid_out,
                                            &tx_wire)) {
                    xQueueSend(ctx->txQueue, &tx_wire, 0);
                }
            }

            gpio_put(TEST_PIN, 0);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(SECS2MSECS(FREQ2PERIOD(task_rate))));
    }
}
