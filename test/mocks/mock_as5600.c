#include <stdint.h>
#include "sensors/as5600.h"

const uint8_t  AS5600_DEFAULT_I2C_ADDR     = 0x36;
const uint8_t  AS5600_CLOCK_WISE           = 0;
const uint8_t  AS5600_COUNTERCLOCK_WISE    = 1;
const uint16_t AS5600_ANGULAR_RESOLUTION   = 4096;
const uint16_t AS5600_RAW_ANGLE_MIN        = 0;
const uint16_t AS5600_RAW_ANGLE_MAX        = 4095;

uint16_t mock_as5600_raw_angle = 0;

uint16_t as5600_get_raw_angle(volatile as5600_t *enc) {
    (void)enc;
    return mock_as5600_raw_angle;
}
