/**
 * @file gimbal_configuration.c
 * @brief Source file for gimbal settings and configurations module.
 */

#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/flash.h"
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

typedef struct {
    const uint8_t *buffer;
    size_t erase_bytes;
    size_t write_bytes;
} flash_write_args_t;

static void flash_write_worker(void *param) {
    flash_write_args_t *args = (flash_write_args_t *)param;
    flash_range_erase(FLASH_TARGET_OFFSET, args->erase_bytes);
    flash_range_program(FLASH_TARGET_OFFSET, args->buffer, args->write_bytes);
}

/* Save gimbal configuration to non-volatile flash memory */
void save_gimbal_configuration(gimbal_configuration_t *config) {
    memcpy(&gc, config, sizeof(gimbal_configuration_t));
    uint8_t *configAsBytes = (uint8_t *)&gc;
    int configSize = sizeof(*config);

    // Pad buffer to an exact multiple of FLASH_PAGE_SIZE (required by flash_range_program)
    int rawSize = configSize + 1;
    int writePages = (rawSize + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    size_t writeBytes = writePages * FLASH_PAGE_SIZE;

    int sectorCount = (writeBytes + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
    size_t eraseBytes = sectorCount * FLASH_SECTOR_SIZE;

    uint8_t *buffer = calloc(1, writeBytes);
    if (buffer == NULL) return;

    memcpy(buffer, configAsBytes, configSize);
    buffer[configSize] = calculate_checksum(configAsBytes, configSize);

    flash_write_args_t args = {
        .buffer      = buffer,
        .erase_bytes = eraseBytes,
        .write_bytes = writeBytes,
    };

    // Safely locks out Core 0 under FreeRTOS SMP, disables IRQs, writes flash, and restores
    flash_safe_execute(flash_write_worker, &args, UINT32_MAX);

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
