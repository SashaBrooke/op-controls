/**
 * @file pinout.c
 * @brief Defines hardware pinout setup behaviour.
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "pinout.h"

/* Setup GPIO pinout */
void setupGPIO() {
    // Configure I2C Communication
    i2c_init(PAN_I2C_PORT, 400000);
    gpio_set_function(PAN_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PAN_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PAN_I2C_SDA_PIN);
    gpio_pull_up(PAN_I2C_SCL_PIN);

    // Configure PWM output
    gpio_set_function(PAN_PWM_PIN, GPIO_FUNC_PWM);

    unsigned int pwmSliceNum = pwm_gpio_to_slice_num(PAN_PWM_PIN);  // Pan and tilt PWM pins are on the same slice
    pwm_config configPWM = pwm_get_default_config();
    pwm_config_set_clkdiv(&configPWM, PWM_CLK_DIVIDER);
    pwm_init(pwmSliceNum, &configPWM, true);
    pwm_set_wrap(pwmSliceNum, PWM_TOP_REG - 1); 

    pwm_set_gpio_level(PAN_PWM_PIN, 0);  // Explicitly set to 0 duty cycle initially
    pwm_set_enabled(pwmSliceNum, true);

    // Configure initial motor directions
    gpio_init(PAN_MOTOR_DIR_PIN);
    gpio_set_dir(PAN_MOTOR_DIR_PIN, GPIO_OUT);
    gpio_put(PAN_MOTOR_DIR_PIN, 0);  // Default LOW, just be explicit

    // Configure limit switch pins
    gpio_init(PAN_LOWER_LIMIT_PIN);
    gpio_set_dir(PAN_LOWER_LIMIT_PIN, GPIO_IN);
    gpio_pull_up(PAN_LOWER_LIMIT_PIN);

    gpio_init(PAN_UPPER_LIMIT_PIN);
    gpio_set_dir(PAN_UPPER_LIMIT_PIN, GPIO_IN);
    gpio_pull_up(PAN_UPPER_LIMIT_PIN);

    // Configure debugging pin
    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0);  // Default LOW, just be explicit
}
