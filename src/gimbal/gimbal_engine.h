#ifndef GIMBAL_ENGINE_H
#define GIMBAL_ENGINE_H

#include "gimbal/gimbal_configuration.h"
#include "controls/pid.h"
#include "sensors/as5600.h"

typedef struct {
    // Axis PID controllers
    volatile pid_controller_t pan_position_controller;

    // Axis encoders
    volatile as5600_t pan_encoder;
} gimbal_engine_t;

void setup_gimbal_engine(gimbal_engine_t *engine, gimbal_configuration_t *config);

#endif // GIMBAL_ENGINE_H
