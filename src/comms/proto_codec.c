#include "comms/proto_codec.h"
#include "pb_encode.h"
#include "pb_decode.h"

static bool encode_packet_to_wire(const opcomms_intcomm_IntcommPacket *pkt, wire_frame_t *out_wire) {
    uint8_t pb_buf[MAX_PB_PAYLOAD_SIZE];
    pb_ostream_t stream = pb_ostream_from_buffer(pb_buf, sizeof(pb_buf));

    if (!pb_encode(&stream, opcomms_intcomm_IntcommPacket_fields, pkt)) {
        return false;
    }
    return packet_frame(pb_buf, stream.bytes_written, out_wire);
}

bool proto_decode_command(const raw_pb_frame_t *raw_pb, opcomms_intcomm_Command *out_cmd) {
    opcomms_intcomm_IntcommPacket pkt = opcomms_intcomm_IntcommPacket_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(raw_pb->data, raw_pb->len);

    if (!pb_decode(&stream, opcomms_intcomm_IntcommPacket_fields, &pkt)) {
        return false;
    }
    if (pkt.which_payload != opcomms_intcomm_IntcommPacket_command_tag) {
        return false;
    }

    *out_cmd = pkt.payload.command;
    return true;
}

bool proto_encode_ack(const opcomms_intcomm_Ack *ack, wire_frame_t *out_wire) {
    opcomms_intcomm_IntcommPacket pkt = opcomms_intcomm_IntcommPacket_init_zero;
    pkt.which_payload = opcomms_intcomm_IntcommPacket_ack_tag;
    pkt.payload.ack   = *ack;
    return encode_packet_to_wire(&pkt, out_wire);
}

bool proto_encode_axis_stream(opcomms_intcomm_Axis axis, float angle_deg,
                              opcomms_intcomm_MotorDirection dir, uint32_t pwm,
                              wire_frame_t *out_wire) {
    opcomms_intcomm_IntcommPacket pkt = opcomms_intcomm_IntcommPacket_init_zero;
    pkt.which_payload = opcomms_intcomm_IntcommPacket_stream_tag;
    pkt.payload.stream.which_data = opcomms_intcomm_Stream_axis_info_tag;

    opcomms_intcomm_AxisInfo *info = &pkt.payload.stream.data.axis_info;
    info->axis      = axis;
    info->angle_deg = angle_deg;
    info->motor_dir = dir;
    info->motor_pwm = pwm;

    return encode_packet_to_wire(&pkt, out_wire);
}

bool proto_encode_pid_stream(opcomms_intcomm_Axis axis, float setpoint, float measured,
                             float p_term, float i_term, float d_term, float output,
                             wire_frame_t *out_wire) {
    opcomms_intcomm_IntcommPacket pkt = opcomms_intcomm_IntcommPacket_init_zero;
    pkt.which_payload = opcomms_intcomm_IntcommPacket_stream_tag;
    pkt.payload.stream.which_data = opcomms_intcomm_Stream_pid_info_tag;

    opcomms_intcomm_PidInfo *pid = &pkt.payload.stream.data.pid_info;
    pid->axis     = axis;
    pid->setpoint = setpoint;
    pid->measured = measured;
    pid->p_term   = p_term;
    pid->i_term   = i_term;
    pid->d_term   = d_term;
    pid->output   = output;

    return encode_packet_to_wire(&pkt, out_wire);
}
