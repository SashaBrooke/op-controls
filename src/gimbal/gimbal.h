#ifndef GIMBAL_H
#define GIMBAL_H

#include "gimbal_configuration.h"
#include "gimbal_engine.h"
#include "gimbal_state.h"

typedef struct {
    gimbal_state_t state;
    gimbal_engine_t engine;
    gimbal_configuration_t config;
} gimbal_t;

void setup_gimbal(gimbal_t *gimbal);

#endif // GIMBAL_H
