#include "gimbal_engine.h"
#include "gimbal_configuration.h"
#include "hardware_configuration.h"
#include "controls/pid.h"
#include "sensors/as5600.h"

void setup_gimbal_engine(gimbal_engine_t *engine, gimbal_configuration_t *config) {
    // Use already setup PID controller from gimbal configuration
    engine->pan_position_controller = config->pan_position_controller;

    // Setup pan encoder
    engine->pan_encoder = as5600_setup(PAN_I2C_PORT, PAN_ENC_DIR_PIN, AS5600_CLOCK_WISE);
}
