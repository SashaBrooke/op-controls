/**
 * @file hardware_configuration.h
 * @brief Defines hardware configuration used for the gimbal control system board.
 * 
 * This configuration currently describes the use of a Raspberry Pi Pico (RP2040).
 */

#ifndef HARDWARE_CONFIGURATION_H
#define HARDWARE_CONFIGURATION_H

// Pan hardware
#define PAN_I2C_PORT             i2c0
#define PAN_I2C_SDA_PIN          4
#define PAN_I2C_SCL_PIN          5
#define PAN_ENC_DIR_PIN          3
#define PAN_PWM_PIN              6
#define PAN_MOTOR_DIR_PIN        8
#define PAN_LOWER_LIMIT_PIN      17
#define PAN_UPPER_LIMIT_PIN      16

// Debug hardware
#define TEST_PIN                 22

// PWM settings
#define PWM_TOP_REG              100
#define PWM_CLK_DIVIDER          125.0f

/**
 * @brief Configures GPIO pins and peripherals for I2C, PWM, and debugging.
 */
void setup_gpio();

#endif // HARDWARE_CONFIGURATION_H