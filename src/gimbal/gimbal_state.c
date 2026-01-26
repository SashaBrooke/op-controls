#include "gimbal_state.h"

// Default gimbal state values
const bool          GIMBAL_DEFAULT_SAVED_CONFIG_FLAG       = true; // No new settings to save on setup
const gimbal_mode_e GIMBAL_DEFAULT_SAFE_MODE               = GIMBAL_MODE_FREE; // Safety: Force free mode on powerup
const float         GIMBAL_DEFAULT_AXIS_VALUE              = 0.0f; // Axis position/velocity
const float         GIMBAL_DEFAULT_MOTOR_PWM               = 0.0f;
const uint8_t       GIMBAL_DEFAULT_MOTOR_DIR               = 0; // TODO: either use as5600 constant or pull from proto file
const float         GIMBAL_DEFAULT_AXIS_SETPOINT           = GIMBAL_DEFAULT_UNSET_ROM;
const float         GIMBAL_DEFAULT_AXIS_SOFT_LIMIT         = GIMBAL_DEFAULT_UNSET_ROM;
const bool          GIMBAL_DEFAULT_GLOBAL_STREAMING_STATUS = false; // Only stream when requested
const uint16_t      GIMBAL_DEFAULT_STREAM_RATE             = 30; // Hz

void setup_gimbal_state(gimbal_state_t *state) {
    state->saved_configuration   = GIMBAL_DEFAULT_SAVED_CONFIG_FLAG;
    state->gimbal_mode           = GIMBAL_DEFAULT_SAFE_MODE;
    state->pan_position          = GIMBAL_DEFAULT_AXIS_VALUE;
    state->pan_position_setpoint = GIMBAL_DEFAULT_AXIS_SETPOINT;
    state->pan_motor_pwm         = GIMBAL_DEFAULT_MOTOR_PWM;
    state->pan_motor_dir         = GIMBAL_DEFAULT_MOTOR_DIR;
    state->pan_lower_limit       = GIMBAL_DEFAULT_AXIS_SOFT_LIMIT;
    state->pan_upper_limit       = GIMBAL_DEFAULT_AXIS_SOFT_LIMIT;
    state->streaming             = GIMBAL_DEFAULT_GLOBAL_STREAMING_STATUS;
    state->stream_rate           = GIMBAL_DEFAULT_STREAM_RATE;
}
