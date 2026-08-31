#ifndef GIMBAL_CONFIGURATION_H
#define GIMBAL_CONFIGURATION_H

#include <stdint.h>
#include <stdbool.h>
#include "controls/pid.h"
#include "software_configuration.h"

// Gimbal serial number is restricted to a uint8 for now
#define GIMBAL_SERIAL_NUMBER_MIN  0
#define GIMBAL_SERIAL_NUMBER_MAX  255

/**
 * @struct gimbal_configuration_t
 * @brief Represents the saved configuration of the gimbal.
 */
typedef struct {
    // Version tracking
    uint8_t serial_number;

    // Controls gains
    volatile pid_controller_t pan_position_controller;
} gimbal_configuration_t;

void setup_gimbal_configuration(gimbal_configuration_t *config);

/**
 * @brief Loads the saved gimbal configuration.
 *
 * Retrieves the a saved gimbal configuration from non-volatile memory.
 *
 * @param config Pointer to the gimbal configuration structure to load into.
 * @return `true` if a configuration was loaded,
 *         `false` if none was found to load.
 */
bool load_gimbal_configuration(gimbal_configuration_t *config);

/**
 * @brief Displays the saved configuration of the gimbal.
 * @param config Pointer to the gimbal configuration structure to display.
 */
void display_gimbal_configuration(gimbal_configuration_t *config);

/**
 * @brief Saves the current gimbal configuration.
 *
 * Stores the gimbal's configuration in non-volatile memory.
 *
 * @param config Pointer to the gimbal configuration structure to save.
 */
void save_gimbal_configuration(gimbal_configuration_t *config);

/**
 * @brief Calculates an XOR checksum for a given data buffer.
 *
 * This function iterates over each byte in the provided data buffer and calculates
 * the checksum by XORing each byte. The resulting checksum is returned as an 8-bit
 * value. This checksum can be used to verify the integrity of the data later.
 *
 * @param[in] data Pointer to the data buffer for which the checksum is calculated.
 * @param[in] size The size (in bytes) of the data buffer.
 *
 * @return The XOR checksum of the provided data buffer as an 8-bit unsigned integer.
 */
uint8_t calculate_checksum(uint8_t* data, int size);

/**
 * @brief Verifies the integrity of the data using the provided checksum.
 *
 * This function extracts the stored checksum from the last byte of the data buffer,
 * recalculates the checksum over the remaining data (excluding the checksum byte),
 * and compares the two. If the recalculated checksum matches the stored checksum,
 * the data is considered valid; otherwise, it is deemed corrupted/modified/invalid.
 *
 * @param[in] data Pointer to the data buffer containing both the data and the checksum.
 * @param[in] sizeWithChecksum The size (in bytes) of the entire data buffer, including the checksum.
 *
 * @return `true` if the checksum verification is successful (i.e., the checksums match),
 *         `false` if the checksum verification fails (i.e., the checksums do not match).
 */
bool verify_checksum(uint8_t *data, int sizeWithChecksum);

#endif // GIMBAL_CONFIGURATION_H
