#include "unity.h"
#include "comms/proto_codec.h"
#include "comms/packet.h"
#include "pb_encode.h"
#include "pb_decode.h"

void setUp(void) {}
void tearDown(void) {}

void test_proto_decode_command_roundtrip(void) {
    opcomms_intcomm_IntcommPacket pkt = opcomms_intcomm_IntcommPacket_init_zero;
    pkt.which_payload = opcomms_intcomm_IntcommPacket_command_tag;
    pkt.payload.command.packet_id = 77;
    pkt.payload.command.which_cmd = opcomms_intcomm_Command_setpoint_command_tag;
    pkt.payload.command.cmd.setpoint_command.axis = opcomms_intcomm_Axis_AXIS_PAN;
    pkt.payload.command.cmd.setpoint_command.setpoint = 1234.5f;

    raw_pb_frame_t raw = {0};
    pb_ostream_t ostream = pb_ostream_from_buffer(raw.data, sizeof(raw.data));
    TEST_ASSERT_TRUE(pb_encode(&ostream, opcomms_intcomm_IntcommPacket_fields, &pkt));
    raw.len = ostream.bytes_written;

    opcomms_intcomm_Command decoded = {0};
    TEST_ASSERT_TRUE(proto_decode_command(&raw, &decoded));
    TEST_ASSERT_EQUAL_UINT32(77, decoded.packet_id);
    TEST_ASSERT_EQUAL(opcomms_intcomm_Command_setpoint_command_tag, decoded.which_cmd);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1234.5f, decoded.cmd.setpoint_command.setpoint);
}

void test_proto_decode_rejects_non_command_packet(void) {
    // Encode an Ack instead of a Command and ensure proto_decode_command rejects it
    opcomms_intcomm_IntcommPacket pkt = opcomms_intcomm_IntcommPacket_init_zero;
    pkt.which_payload = opcomms_intcomm_IntcommPacket_ack_tag;
    pkt.payload.ack.packet_id = 5;

    raw_pb_frame_t raw = {0};
    pb_ostream_t ostream = pb_ostream_from_buffer(raw.data, sizeof(raw.data));
    TEST_ASSERT_TRUE(pb_encode(&ostream, opcomms_intcomm_IntcommPacket_fields, &pkt));
    raw.len = ostream.bytes_written;

    opcomms_intcomm_Command decoded = {0};
    TEST_ASSERT_FALSE(proto_decode_command(&raw, &decoded));
}

void test_proto_encode_ack_roundtrip(void) {
    opcomms_intcomm_Ack ack = opcomms_intcomm_Ack_init_zero;
    ack.packet_id = 42;
    ack.code = opcomms_intcomm_AckCode_STATUS_OUT_OF_LIMITS;

    wire_frame_t wire = {0};
    TEST_ASSERT_TRUE(proto_encode_ack(&ack, &wire));

    raw_pb_frame_t raw = {0};
    TEST_ASSERT_TRUE(packet_unframe(wire.data, wire.len - 1, &raw));

    opcomms_intcomm_IntcommPacket rx = opcomms_intcomm_IntcommPacket_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(raw.data, raw.len);
    TEST_ASSERT_TRUE(pb_decode(&istream, opcomms_intcomm_IntcommPacket_fields, &rx));
    TEST_ASSERT_EQUAL(opcomms_intcomm_IntcommPacket_ack_tag, rx.which_payload);
    TEST_ASSERT_EQUAL_UINT32(42, rx.payload.ack.packet_id);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OUT_OF_LIMITS, rx.payload.ack.code);
}

void test_proto_encode_stream_packets_roundtrip(void) {
    wire_frame_t wire = {0};
    raw_pb_frame_t raw = {0};
    opcomms_intcomm_IntcommPacket rx = opcomms_intcomm_IntcommPacket_init_zero;

    // AxisInfo
    TEST_ASSERT_TRUE(proto_encode_axis_stream(
        opcomms_intcomm_Axis_AXIS_PAN, 95.25f,
        opcomms_intcomm_MotorDirection_MOTOR_DIR_CW, 310, &wire));
    TEST_ASSERT_TRUE(packet_unframe(wire.data, wire.len - 1, &raw));

    pb_istream_t istream = pb_istream_from_buffer(raw.data, raw.len);
    TEST_ASSERT_TRUE(pb_decode(&istream, opcomms_intcomm_IntcommPacket_fields, &rx));
    TEST_ASSERT_EQUAL(opcomms_intcomm_Stream_axis_info_tag, rx.payload.stream.which_data);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 95.25f, rx.payload.stream.data.axis_info.angle_deg);
    TEST_ASSERT_EQUAL_UINT32(310, rx.payload.stream.data.axis_info.motor_pwm);

    // PidInfo
    TEST_ASSERT_TRUE(proto_encode_pid_stream(
        opcomms_intcomm_Axis_AXIS_PAN, 100.0f, 95.25f, 4.0f, 1.0f, 0.25f, 5.25f, &wire));
    TEST_ASSERT_TRUE(packet_unframe(wire.data, wire.len - 1, &raw));

    rx = (opcomms_intcomm_IntcommPacket)opcomms_intcomm_IntcommPacket_init_zero;
    istream = pb_istream_from_buffer(raw.data, raw.len);
    TEST_ASSERT_TRUE(pb_decode(&istream, opcomms_intcomm_IntcommPacket_fields, &rx));
    TEST_ASSERT_EQUAL(opcomms_intcomm_Stream_pid_info_tag, rx.payload.stream.which_data);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.25f, rx.payload.stream.data.pid_info.output);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_proto_decode_command_roundtrip);
    RUN_TEST(test_proto_decode_rejects_non_command_packet);
    RUN_TEST(test_proto_encode_ack_roundtrip);
    RUN_TEST(test_proto_encode_stream_packets_roundtrip);
    return UNITY_END();
}
