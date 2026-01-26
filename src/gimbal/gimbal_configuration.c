/**
 * @file gimbal_configuration.c
 * @brief Source file for gimbal settings and configurations module.
 */

#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "gimbal_configuration.h"
#include "sensors/as5600.h"
#include "utils/time_utils.h"
#include "software_configuration.h"

#define FLASH_TARGET_OFFSET (256 * 1024) // Starts at 256K

static gimbal_configuration_t gc; // Define a config at a constant location in memory for flash programming

void setup_gimbal_configuration(gimbal_configuration_t *config) {
    bool loadedPrevConfig = load_gimbal_configuration(config);

    if (loadedPrevConfig) {
        // Setup controllers using saved values
        config->pan_position_controller = pid_setup(
            config->pan_position_controller.Kp,
            config->pan_position_controller.Ki,
            config->pan_position_controller.Kd,
            config->pan_position_controller.tau,
            config->pan_position_controller.outLimMin,
            config->pan_position_controller.outLimMax,
            config->pan_position_controller.intLimMin,
            config->pan_position_controller.intLimMax,
            FREQ2PERIOD(CONTROLS_FREQ),
            (float)AS5600_ANGULAR_RESOLUTION
        );
    } else {
        // Setup using default values
        // printf("Creating new configuration using default values.\n");
        config->serial_number = 0;
        config->pan_position_controller = pid_setup(
            0.0f, 0.0f, 0.0f,
            0.0f,
            -100.0f, 100.0f,
            0.0f, 0.0f,
            FREQ2PERIOD(CONTROLS_FREQ),
            (float)AS5600_ANGULAR_RESOLUTION
        );
    }
}

/* Load gimbal configuration from non-volatile flash memory */
bool load_gimbal_configuration(gimbal_configuration_t *config) {
    if (config == NULL) {
        return false;
    }

    // Calculate the address in flash memory
    const uint8_t* flash_target_contents = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);

    // Create a buffer to hold the configuration data and the checksum
    int configSize = sizeof(gimbal_configuration_t);
    int bufferSize = configSize + 1;
    uint8_t *buffer = malloc(bufferSize);

    memcpy(buffer, flash_target_contents, bufferSize);

    if (verify_checksum(buffer, bufferSize)) {
        // If the checksum is valid, copy the saved configuration into the provided structure
        memcpy(config, buffer, configSize);
        free(buffer);
        // printf("Gimbal configuration loaded successfully.\n");
        // TODO: queue error packet
        return true;
    } else {
        free(buffer);
        // printf("No configuration to load.\n");
        // TODO: queue error packet
        return false;
    }
}

/* Save gimbal configuration to non-volatile flash memory */
void save_gimbal_configuration(gimbal_configuration_t *config) {
    memcpy(&gc, config, sizeof(gimbal_configuration_t));
    uint8_t* configAsBytes = (uint8_t*) &gc;
    int configSize = sizeof(*config);

    // Create a buffer to hold the configuration and a computed checksum
    int bufferSize = configSize + 1;
    uint8_t *buffer = malloc(bufferSize);
    memcpy(buffer, configAsBytes, configSize);

    uint8_t checksum = calculate_checksum(configAsBytes, configSize);
    buffer[configSize] = checksum;  // Append the checksum as the last byte
    
    // Calculate how many flash pages we're gonna need to write
    int writeSize = (bufferSize / FLASH_PAGE_SIZE) + 1;

    // Calculate how many flash sectors we're gonna need to erase
    int sectorCount = ((writeSize * FLASH_PAGE_SIZE) / FLASH_SECTOR_SIZE) + 1;

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE * sectorCount);
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE * writeSize);
    restore_interrupts(interrupts);

    free(buffer);
}

/* Calculate XOR checksum */
uint8_t calculate_checksum(uint8_t* data, int size) {
    uint8_t checksum = 0;
    for (int i = 0; i < size; i++) {
        checksum ^= data[i];  // XOR checksum for simplicity
    }
    return checksum;
}

/* Verify data (wth an included checksum) integrity */
bool verify_checksum(uint8_t *data, int sizeWithChecksum) {
    int configSize = sizeWithChecksum - 1;  // Exclude the checksum byte
    uint8_t storedChecksum = data[configSize];  // Extract stored checksum
    uint8_t calculatedChecksum = calculate_checksum(data, configSize);  // Recalculate checksum
    return (storedChecksum == calculatedChecksum);
}
