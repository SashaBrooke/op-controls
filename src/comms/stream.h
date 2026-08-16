#ifndef STREAM_H
#define STREAM_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "packet.h"

#include "extcomm.pb.h"

#define STREAM_PACKET_ID_IGNORED 0u // Stream packets do not need to specify a packet ID

/**
 * @brief Encodes an AxisInfo stream packet into the provided packet_t.
 *
 * @param[out] packet      Pointer to the packet_t to fill with encoded data.
 * @param[in]  axis        Axis value (e.g., AXIS_PAN, AXIS_TILT).
 * @param[in]  angle_deg   Angle in degrees for the axis.
 * @param[in]  motor_dir   Motor direction (CW, CCW, UNKNOWN).
 * @param[in]  motor_pwm   Motor PWM value.
 * @return Number of bytes written to packet->data, or 0 on error.
 */
size_t encode_axis_information(packet_t *packet,
    opcomms_extcomm_Axis axis, float angle_deg,
    opcomms_extcomm_MotorDirection motor_dir, uint32_t motor_pwm);

#endif // STREAM_H
