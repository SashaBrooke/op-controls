#include <stdbool.h>
#include <stddef.h>
#include "gimbal/gimbal_configuration.h"

bool mock_config_saved = false;
gimbal_configuration_t mock_last_saved_config = {0};

void save_gimbal_configuration(gimbal_configuration_t *config) {
    mock_config_saved = true;
    if (config != NULL) {
        mock_last_saved_config = *config;
    }
}
