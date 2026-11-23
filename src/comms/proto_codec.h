#ifndef PROTO_CODEC_H
#define PROTO_CODEC_H

#include <stdbool.h>
#include "intcomm.pb.h"
#include "comms/packet.h"

// Decode raw Protobuf bytes into an IntcommPacket (returns true only if it contains a Command)
bool proto_decode_command(const raw_pb_frame_t *raw_pb, opcomms_intcomm_Command *out_cmd);

// Encode an Ack message into a COBS-framed wire packet ready for txQueue
bool proto_encode_ack(const opcomms_intcomm_Ack *ack, wire_frame_t *out_wire);

// Encode an AxisInfo stream message into a COBS-framed wire packet
bool proto_encode_axis_stream(opcomms_intcomm_Axis axis, float angle_deg,
                              opcomms_intcomm_MotorDirection dir, uint32_t pwm,
                              wire_frame_t *out_wire);

// Encode a PidInfo stream message into a COBS-framed wire packet
bool proto_encode_pid_stream(opcomms_intcomm_Axis axis, float setpoint, float measured,
                             float p_term, float i_term, float d_term, float output,
                             wire_frame_t *out_wire);

#endif // PROTO_CODEC_H
