/**
 * @file AS5600.h
 * @brief Library module for the AS5600 magnetic rotary encoder.
 *
 * This library provides an interface for communicating with the AS5600 magnetic rotary encoder 
 * via I2C.
 * 
 * @note This library is specifically tailored for use of the AS5600 with Raspberry Pi Pico
 *       boards and requires the Pico C/C++ SDK.
 */

#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>

#include "hardware/i2c.h"

#include "utils/rotary_utils.h"

// Conversions
#define AS5600_RAW_TO_DEGREES(raw) ((raw) * (MAX_DEGREES / AS5600_ANGULAR_RESOLUTION))
#define AS5600_DEGREES_TO_RAW(deg) ((deg) * (AS5600_ANGULAR_RESOLUTION / MAX_DEGREES))

// Address
extern const uint8_t   AS5600_DEFAULT_I2C_ADDR;

//  Directions
extern const uint8_t   AS5600_CLOCK_WISE;
extern const uint8_t   AS5600_COUNTERCLOCK_WISE;

//  Resolution
extern const uint16_t  AS5600_ANGULAR_RESOLUTION;
extern const uint16_t  AS5600_RAW_ANGLE_MIN;
extern const uint16_t  AS5600_RAW_ANGLE_MAX;

/**
 * @struct as5600_t
 * @brief Structure representing the AS5600 encoder.
 */
typedef struct {
    // Initialisation
    bool initialised;

    // Hardware
    i2c_inst_t *i2c;
    uint8_t dir_pin;

    // Offset
    uint16_t offset;
} as5600_t;

/**
 * @brief Sets up the AS5600 encoder.
 *
 * Initializes the AS5600 encoder with the specified I2C instance, direction pin, and rotation direction.
 *
 * @param i2c Pointer to the I2C instance.
 * @param dir_pin GPIO pin used for direction control.
 * @param direction Rotation direction (AS5600_CLOCK_WISE or AS5600_COUNTERCLOCK_WISE).
 * @return A configured as5600_t structure.
 */
as5600_t as5600_setup(i2c_inst_t *i2c, uint8_t dir_pin, uint8_t direction);
/**
 * @brief Checks if the AS5600 encoder is connected.
 *
 * @param enc Pointer to the as5600 encoder structure.
 * @return True if the encoder is connected, false otherwise.
 */
bool as5600_is_connected(volatile as5600_t *enc);

/**
 * @brief Reads the status register of the AS5600 encoder.
 *
 * @param enc Pointer to the as5600 encoder structure.
 * @return Status register value.
 */
uint8_t as5600_read_status(volatile as5600_t *enc);

/**
 * @brief Checks if a magnet is detected by the encoder.
 *
 * @param enc Pointer to the as5600 encoder structure.
 * @return True if a magnet is detected, false otherwise.
 */
bool as5600_magnet_detected(volatile as5600_t *enc);

/**
 * @brief Checks if the detected magnet is too weak.
 *
 * @param enc Pointer to the as5600 encoder structure.
 * @return True if the magnet is too weak, false otherwise.
 */
bool as5600_magnet_too_weak(volatile as5600_t *enc);

/**
 * @brief Checks if the detected magnet is too strong.
 *
 * @param enc Pointer to the as5600 encoder structure.
 * @return True if the magnet is too strong, false otherwise.
 */
bool as5600_magnet_too_strong(volatile as5600_t *enc);

/**
 * @brief Checks if the detected magnet is within the optimal range.
 *
 * @param enc Pointer to the as5600 encoder structure.
 * @return True if the magnet is good, false otherwise.
 */
bool as5600_magnet_good(volatile as5600_t *enc);

/**
 * @brief Reads the automatic gain control (AGC) value of the encoder.
 *
 * @param enc Pointer to the as5600 encoder structure.
 * @return AGC value.
 */
uint8_t as5600_read_agc(volatile as5600_t *enc);

/**
 * @brief Reads the raw angle value from the encoder.
 *
 * @param enc Pointer to the as5600 encoder structure.
 * @return Raw angle value.
 */
uint16_t as5600_get_raw_angle(volatile as5600_t *enc);

#endif // AS5600_H
