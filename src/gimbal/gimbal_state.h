#ifndef GIMBAL_STATE_H
#define GIMBAL_STATE_H

#include <stdint.h>
#include <stdbool.h>

// Gimbal range of motion (ROM)
#define GIMBAL_DEFAULT_UNSET_ROM  (-1)
#define GIMBAL_360_ROM            (-2)

/**
 * @enum gimbal_mode_e
 * @brief Operational modes for the gimbal.
 */
typedef enum {
    GIMBAL_MODE_LOWER_LIMIT,
    GIMBAL_MODE_FREE,
    GIMBAL_MODE_ARMED,
    GIMBAL_MODE_UPPER_LIMIT
} gimbal_mode_e;

/**
 * @struct gimbal_t
 * @brief Represents the runtime state of the gimbal.
 */
typedef struct {
    // Saved flag
    bool saved_configuration;

    // Operational mode
    gimbal_mode_e gimbal_mode;

    // Axis information
    float pan_position;
    float pan_position_setpoint;

    // TODO: velocity control
    // float pan_velocity;
    // float pan_velocity_setpoint;

    // Axis motor information
    float pan_motor_pwm;
    uint8_t pan_motor_dir;

    // Axis soft limits
    float pan_lower_limit; // TODO: limit_t struct
    float pan_upper_limit;

    // Global stream
    bool streaming;
    uint16_t stream_rate;
} gimbal_state_t;

void setup_gimbal_state(gimbal_state_t *state);

#endif // GIMBAL_STATE_H
