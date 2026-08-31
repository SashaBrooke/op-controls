#include "gimbal.h"

/* Set up gimbal with default settings */
void setup_gimbal(gimbal_t *gimbal) {
    // Load or setup new gimbal configuration
    gimbal_configuration_t *config = &gimbal->config;
    setup_gimbal_configuration(config);

    // Setup gimbal engine
    gimbal_engine_t *engine = &gimbal->engine;
    setup_gimbal_engine(engine, config);

    // Setup gimbal runtime state
    gimbal_state_t *state = &gimbal->state;
    setup_gimbal_state(state);
}
