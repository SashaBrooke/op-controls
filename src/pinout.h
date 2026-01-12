/**
 * @file pinout.h
 * @brief Defines hardware pinout used for the gimbal control system board.
 * 
 * This pinout currently describes the use of a Raspberry Pi Pico (RP2040).
 */

#ifndef PINOUT_H
#define PINOUT_H

// Pan hardware
#define PAN_I2C_PORT             i2c0
#define PAN_I2C_SDA_PIN          4
#define PAN_I2C_SCL_PIN          5
#define PAN_ENC_DIR_PIN          3
#define PAN_PWM_PIN              6
#define PAN_MOTOR_DIR_PIN        8
#define PAN_LOWER_LIMIT_PIN      17
#define PAN_UPPER_LIMIT_PIN      16

// UART hardware
#define UART_ID                  uart0
#define UART_BAUDRATE            115200
#define UART_TX_PIN              0
#define UART_RX_PIN              1

// Debug hardware
#define TEST_PIN                 22

// PWM settings
#define PWM_TOP_REG              100
#define PWM_CLK_DIVIDER          125.0f

/**
 * @brief Configures GPIO pins and peripherals for I2C, UART, PWM, and debugging.
 */
void setupGPIO();

#endif  // PINOUT_H
