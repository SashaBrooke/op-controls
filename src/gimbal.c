/**
 * @file gimbal.c
 * @brief Source file for gimbal controls system module.
 * 
 * This file contains the main entry point for the control board firmware.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "pid.h"
#include "as5600.h"
#include "command.h"
#include "gimbal_configuration.h"
#include "rotary_utils.h"
#include "pinout.h"

#include "extcomm.pb.h"
#include "pb_encode.h"

#define FREQ2PERIOD(freq) ((freq) != 0 ? (1.0f / (freq)) : 0.0f) // Converts a frequency in Hz to a period in seconds
#define SECS2MSECS(secs)  ((secs) * 1000)                        // Converts a time from seconds to milliseconds
#define SECS2USECS(secs)  ((secs) * 1000000)                     // Converts a time from seconds to micro-seconds

// Controls loop frequency
#define CONTROLS_FREQ            1000

// RTOS variables
#define CMD_QUEUE_LEN            8
#define CMD_MAX_LEN              100

// Homing
#define HOMING_SPEED             0.1  // (angular position / second)
#define HOMING_DISTANCE          40   // (angular position)
#define HOMING_TIMEOUT           10   // (seconds)
#define SOFT_LIMIT_OFFSET_ANGLE  5    // (degrees)

// Stream variables
volatile uint16_t panPos       = 0;
volatile float panMotor        = 0.0f;
volatile uint8_t panDir        = 0;

volatile bool printFlag = false;

static uint32_t streamSeq = 0;
static uint32_t streamSeqEpoch = 0;

// RTOS structures
static QueueHandle_t cmdQueue = NULL;

/**
 * @brief Set soft limits for each gimbal axis.
 */
void setupAxisLimits(gimbal_t *gimbal) {
    uint64_t startTime = time_us_64();

    // Temp limits (gimbal limits only set if homing process actually completes)
    float tmpPanLowerLimit;
    float tmpPanUpperLimit;

    // Arm gimbal
    gimbal->panPositionSetpoint = (float)panPos;
    gimbal->gimbalMode = GIMBAL_MODE_ARMED;

    // Find pan lower limit
    while (true) {
        bool atPanLowerLimit = !gpio_get(PAN_LOWER_LIMIT_PIN);  // Pulled high default (therefore !)

        if (time_us_64() - startTime >= SECS2USECS(HOMING_TIMEOUT)) {
            printf("Timeout: Homing process exceeded time limit. Exiting homing process.\n");
            gimbal->gimbalMode = GIMBAL_MODE_FREE;
            return;
        }

        if (atPanLowerLimit) {
            tmpPanLowerLimit = (float)panPos;
            printf("#### Lower limit found at %.1f ####\n", tmpPanLowerLimit);
            break;
        } else {
            float newPanSetpoint = gimbal->panPositionSetpoint - HOMING_SPEED;
            gimbal->panPositionSetpoint = newPanSetpoint >= 0 ? newPanSetpoint : newPanSetpoint + AS5600_RAW_ANGLE_MAX;
            printf("Moving to lower pan limit (actual: %u, setpoint: %.1f)\n", panPos, gimbal->panPositionSetpoint);  // Hacky
        }
    }

    sleep_ms(1000);

    // Find pan upper limit
    int iter = 0;
    while (true) {
        bool atPanUpperLimit = !gpio_get(PAN_UPPER_LIMIT_PIN);  // Pulled high default (therefore !)

        if (time_us_64() - startTime >= SECS2USECS(HOMING_TIMEOUT)) {
            printf("Timeout: Homing process exceeded time limit. Exiting homing process.\n");
            gimbal->gimbalMode = GIMBAL_MODE_FREE;
            return;
        }

        if (atPanUpperLimit) {
            tmpPanUpperLimit = (float)panPos;
            printf("#### Upper limit found at %.1f ####\n", tmpPanUpperLimit);
            break;
        } else {
            float newPanSetpoint = gimbal->panPositionSetpoint + HOMING_SPEED;
            gimbal->panPositionSetpoint = newPanSetpoint <= AS5600_RAW_ANGLE_MAX ? newPanSetpoint : newPanSetpoint - AS5600_RAW_ANGLE_MAX;
            printf("Moving to upper pan limit (actual: %u, setpoint: %.1f)\n", panPos, gimbal->panPositionSetpoint);  // Hacky
        }
    }

    sleep_ms(1000);

    // Move to a central position
    float panMidpoint;
    if (tmpPanLowerLimit == tmpPanUpperLimit) {
        panMidpoint = tmpPanLowerLimit;
    } else if (tmpPanLowerLimit < tmpPanUpperLimit) {
        panMidpoint = (tmpPanLowerLimit + tmpPanUpperLimit) / 2;
    } else {
        panMidpoint = fmod(tmpPanLowerLimit + (AS5600_RAW_ANGLE_MAX + tmpPanUpperLimit - tmpPanLowerLimit) / 2, AS5600_RAW_ANGLE_MAX);
    }
    gimbal->panPositionSetpoint = panMidpoint >= 0 ? panMidpoint : panMidpoint + AS5600_RAW_ANGLE_MAX;
    printf("Midpoint: %.0f\n", gimbal->panPositionSetpoint);

    while (true) {
        if (time_us_64() - startTime >= SECS2USECS(HOMING_TIMEOUT)) {
            printf("Timeout: Homing process exceeded time limit. Exiting homing process.\n");
            gimbal->gimbalMode = GIMBAL_MODE_FREE;
            return;
        }

        if (fabs((float)panPos - panMidpoint) < HOMING_DISTANCE) {
            printf("At midpoint (%u actual vs %.1f setpoint)\n", panPos, panMidpoint);

            float gimbalRom = GIMBAL_DEFAULT_UNSET_ROM;
            calculate_rotary_error(&gimbalRom, tmpPanLowerLimit, tmpPanUpperLimit, AS5600_ANGULAR_RESOLUTION);

            printf("%.1f\n", 2 * round(SOFT_LIMIT_OFFSET_ANGLE * AS5600_DEGREES_TO_RAW));
            
            // If the gimbal has sufficient range of motion to apply offsets, actually set soft limits
            if (gimbalRom > 2 * round(SOFT_LIMIT_OFFSET_ANGLE * AS5600_DEGREES_TO_RAW)) {
                gimbal->panLowerLimit = tmpPanLowerLimit + round(SOFT_LIMIT_OFFSET_ANGLE * AS5600_DEGREES_TO_RAW);
                gimbal->panUpperLimit = tmpPanUpperLimit - round(SOFT_LIMIT_OFFSET_ANGLE * AS5600_DEGREES_TO_RAW);
            } else {
                printf("Error: Gimbal range of motion not sufficient to set soft limit safety offsets. Exiting homing process.\n");
                gimbal->gimbalMode = GIMBAL_MODE_FREE;
                return;
            }

            sleep_ms(500);  // Pause at midpoint

            // Disarm
            gimbal->gimbalMode = GIMBAL_MODE_FREE;
            break;
        } else {
            printf("Moving to midpoint (actual: %u, setpoint: %.1f)\n", panPos, gimbal->panPositionSetpoint);  // Hacky
        }
    }

    printf("Finished setting axis soft limits.\n");
}

/**
 * @brief Performs the gimbal control system loop.
 * 
 * @param gimbal Pointer to the gimbal state structure.
 */
void updateMotors(gimbal_t *gimbal) {
    // Timing debug
    gpio_put(TEST_PIN, 1);

    // Pan
    uint16_t panRawAngle = AS5600_getRawAngle(&gimbal->panEncoder);
    panPos = panRawAngle;

    if (gimbal->gimbalMode == GIMBAL_MODE_ARMED) {
        float panPidOutput = PID_update(&gimbal->panPositionController, gimbal->panPositionSetpoint,
            (float)panRawAngle, gimbal->panLowerLimit, gimbal->panUpperLimit);
        float panPwmDutyCycle = PID_normaliseOutput(&gimbal->panPositionController, -100.0f, 100.0f);
        uint8_t panDirection = panPidOutput > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
        gpio_put(PAN_MOTOR_DIR_PIN, panDirection);
        pwm_set_gpio_level(PAN_PWM_PIN, (uint16_t)(abs(panPidOutput)));

        panMotor = panPwmDutyCycle;
        panDir = panDirection;
    } else if (gimbal->gimbalMode = GIMBAL_MODE_FREE) {
        PID_reset(&gimbal->panPositionController);
        gpio_put(PAN_MOTOR_DIR_PIN, 0);      // Default dir
        pwm_set_gpio_level(PAN_PWM_PIN, 0);  // Default pwm

        panMotor = 0;
        panDir = 0;
    }

    // Handle print flag toggling
    static int counter = 0; 
    counter++;
    if (counter >= gimbal->streamRate) {
        printFlag = true;
        counter = 0;
    }

    // Timing debug
    gpio_put(TEST_PIN, 0);
}

static void controlTask(void *arg) {
    gimbal_t *g = (gimbal_t *)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        updateMotors(g);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SECS2MSECS(FREQ2PERIOD(CONTROLS_FREQ))));
    }
}

static void commsTask(void *arg) {
    (void)arg;
    for (;;) {
        char *cmd = readSerialCommand_nonBlocking();
        if (cmd) {
            xQueueSend(cmdQueue, &cmd, 0);
            resetSerialCommandInput();
        }
    }
}

static void commandTask(void *arg) {
    gimbal_bundle_t *bundle = (gimbal_bundle_t *)arg;
    char *cmd;
    for (;;) {
        if (xQueueReceive(cmdQueue, &cmd, portMAX_DELAY) == pdPASS) {
            processCommands(cmd, bundle->gimbal, bundle->config);
        }
    }
}

// Should be stopped and started by command processor (if streaming enabled)
static void streamTask(void *arg) {
    gimbal_t *g = (gimbal_t *)arg;
    TickType_t last = xTaskGetTickCount();
    
    // Buffer for encoded message (max message size + 5 bytes for length prefix)
    // uint8_t buffer[256];
    
    for (;;) {
        // opcomms_extcomm_StreamPacket packet = opcomms_extcomm_StreamPacket_init_zero;

        // packet.has_header = true;
        // packet.header.version = opcomms_extcomm_SchemaVersion_SCHEMA_VERSION;
        // packet.header.seq = streamSeq;
        // packet.header.seq_epoch = streamSeqEpoch;

        // opcomms_extcomm_AxisInfo axis_info = opcomms_extcomm_AxisInfo_init_zero;
        // axis_info.axis = opcomms_extcomm_Axis_AXIS_PAN;
        // axis_info.angle_deg = (float)panPos * AS5600_RAW_TO_DEGREES;

        // // Map panDir to MotorDirection enum
        // if (panDir == AS5600_CLOCK_WISE) {
        //     axis_info.motor_direction = opcomms_extcomm_MotorDirection_MOTOR_DIR_CW;
        // } else if (panDir == AS5600_COUNTERCLOCK_WISE) {
        //     axis_info.motor_direction = opcomms_extcomm_MotorDirection_MOTOR_DIR_CCW;
        // } else {
        //     axis_info.motor_direction = opcomms_extcomm_MotorDirection_MOTOR_DIR_UNKNOWN;
        // }

        // packet.which_payload = opcomms_extcomm_StreamPacket_axis_info_tag;
        // packet.payload.axis_info = axis_info;

        // pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        // if (pb_encode_delimited(&stream, opcomms_extcomm_StreamPacket_fields, &packet)) {
        //     uart_write_nonblocking(buffer, stream.bytes_written);
        //     streamSeq++;
        //     if (streamSeq == 0) {
        //         streamSeqEpoch++;
        //     }
        // } else {
        //     // Encoding failed
        // }

        gpio_put(TEST_PIN, 1);
        
        // // UART write a 3-byte payload byte-by-byte
        // const char *msg = "0\r\n";

        // for (int i = 0; i < 3; i++) {
        //     while (!uart_is_writable(UART_ID)) {
        //         taskYIELD();
        //     }
        //     uart_putc_raw(UART_ID, msg[i]);
        // }
        
        const char *msg = "01234567890123456789012345678901234567890123456789012345678912\r\n";
        fwrite(msg, 1, 64, stdout);

        // uint16_t panRawAngle = AS5600_getRawAngle(&g->panEncoder);
        gpio_put(TEST_PIN, 0);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}

int main() {
    stdio_init_all();

    setupGPIO();

    sleep_ms(3000); // Allow serial monitor to auto-detect COM port
                    // (recommended Arduino IDE for serial monitor/plotter)

    printf("\n ------------------ STARTING GIMBAL ------------------ \n");

    gimbal_t gimbal;
    setupGimbal(&gimbal);

    gimbal_configuration_t gimbalConfig;
    bool loadedPrevConfig = loadGimbalConfiguration(&gimbalConfig);

    if (loadedPrevConfig) {
        // Setup controllers using saved values
        gimbal.panPositionController = PID_setup(gimbalConfig.panPositionController.Kp,
                                                 gimbalConfig.panPositionController.Ki,
                                                 gimbalConfig.panPositionController.Kd,
                                                 gimbalConfig.panPositionController.tau,
                                                 gimbalConfig.panPositionController.outLimMin,
                                                 gimbalConfig.panPositionController.outLimMax,
                                                 gimbalConfig.panPositionController.intLimMin,
                                                 gimbalConfig.panPositionController.intLimMax,
                                                 FREQ2PERIOD(CONTROLS_FREQ),
                                                 (float)AS5600_ANGULAR_RESOLUTION);
    } else {
        // Setup using default values
        printf("Creating new configuration using default values.\n");
        gimbalConfig.serialNumber = 0;
        gimbal.panPositionController = PID_setup(0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 -100.0f,
                                                 100.0f,
                                                 0.0f,
                                                 0.0f,
                                                 FREQ2PERIOD(CONTROLS_FREQ),
                                                 (float)AS5600_ANGULAR_RESOLUTION
                                                 );
    }

    gimbalConfig.panPositionController = gimbal.panPositionController;

    displayGimbalConfiguration(&gimbalConfig);

    gimbal.panEncoder = AS5600_setup(PAN_I2C_PORT, PAN_ENC_DIR_PIN, AS5600_CLOCK_WISE);

    displayGimbal(&gimbal);

    printf("Starting controls loop\n");
    // repeating_timer_t timer;  // Consider using real time clock peripheral if for use longer than ~72 mins
    // add_repeating_timer_us(-SECS2USECS(FREQ2PERIOD(CONTROLS_FREQ)), updateMotors, &gimbal, &timer);

    // sleep_ms(1000);
    // setupAxisLimits(&gimbal);
    // sleep_ms(1000);

    // Enable serial commands
    resetSerialCommandInput();
    char* command;

    // Setup RTOS structures and tasks
    cmdQueue = xQueueCreate(CMD_QUEUE_LEN, sizeof(char *));
    gimbal_bundle_t bundle = { .gimbal = &gimbal, .config = &gimbalConfig };

    xTaskCreate(controlTask, "ctrl", 512, bundle.gimbal, configMAX_PRIORITIES - 1, NULL);
    // xTaskCreate(commsTask,   "com",  512, NULL,           2,                       NULL);
    // xTaskCreate(commandTask, "cmd",  768, &bundle,        2,                       NULL);
    xTaskCreate(streamTask,  "str",  512, bundle.gimbal, 1,                        NULL);

    vTaskStartScheduler();
    for (;;);

    // while(true) {
    //     if (gimbal.streaming && printFlag) {
    //         // Handle specific stream outputs
    //         if (gimbal.panPositionStream) {
    //             printf("%u,", panPos);
                
    //             if (gimbal.gimbalMode == GIMBAL_MODE_ARMED) {
    //                 printf("%f,", gimbal.panPositionSetpoint);
    //             }
    //         }
    //         if (gimbal.panPidStream) {
    //             printf("%f,%f,%f,%f,",
    //                 gimbal.panPositionController.prevError,  // Actually the error from the current controls loop
    //                 gimbal.panPositionController.integrator,
    //                 gimbal.panPositionController.differentiator,
    //                 gimbal.panPositionController.output
    //             );
    //         }
    //         if (gimbal.panMotorStream) {
    //             printf("%f,%u,", panMotor, panDir);
    //         }
    //         printf("\n");

    //         printFlag = false;  // Reset print flag
    //     }

    //     command = readSerialCommand_nonBlocking();
    //     if (command != NULL) {
    //         // cancel_repeating_timer(&timer);
    //         processCommands(command, &gimbal, &gimbalConfig);
    //         // add_repeating_timer_us(-SECS2USECS(FREQ2PERIOD(CONTROLS_FREQ)), updateMotors, &gimbal, &timer);
    //         resetSerialCommandInput();
    //     }
    // }

    return 0;
}
