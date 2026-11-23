#include "unity.h"
#include "comms/command.h"
#include "comms/packet.h"
#include "comms/proto_codec.h"
#include "pb_encode.h"
#include "pb_decode.h"

// External mock controls defined in test/mocks/
extern uint16_t mock_as5600_raw_angle;
extern bool mock_config_saved;
extern gimbal_configuration_t mock_last_saved_config;

static gimbal_t gimbal;

void setUp(void) {
    gimbal = (gimbal_t){0};
    setup_gimbal_state(&gimbal.state);
    gimbal.engine.pan_position_controller = pid_setup(
        1.0f, 0.1f, 0.01f, 0.05f, -100.0f, 100.0f, -50.0f, 50.0f, 0.001f, 4096.0f
    );
    gimbal.config.pan_position_controller = gimbal.engine.pan_position_controller;

    // Reset mocks before each test
    mock_as5600_raw_angle = 0;
    mock_config_saved = false;
    mock_last_saved_config = (gimbal_configuration_t){0};
}

void tearDown(void) {}

void test_arm_fails_when_limits_unset(void) {
    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    cmd.packet_id = 10;
    cmd.which_cmd = opcomms_intcomm_Command_system_action_tag;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_GIMBAL_ARM;

    opcomms_intcomm_Ack ack;
    command_dispatch(&cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL_UINT32(10, ack.packet_id);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_UNSET_LIMITS, ack.code);
    TEST_ASSERT_EQUAL(GIMBAL_MODE_FREE, gimbal.state.gimbal_mode);
}

void test_arm_fails_when_encoder_out_of_limits(void) {
    gimbal.state.pan_lower_limit = 1000.0f;
    gimbal.state.pan_upper_limit = 2000.0f;
    mock_as5600_raw_angle = 2500; // Outside [1000, 2000]

    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    cmd.packet_id = 11;
    cmd.which_cmd = opcomms_intcomm_Command_system_action_tag;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_GIMBAL_ARM;

    opcomms_intcomm_Ack ack;
    command_dispatch(&cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OUT_OF_LIMITS, ack.code);
    TEST_ASSERT_EQUAL(GIMBAL_MODE_FREE, gimbal.state.gimbal_mode);
}

void test_arm_succeeds_and_locks_current_angle_as_setpoint(void) {
    gimbal.state.pan_lower_limit = 1000.0f;
    gimbal.state.pan_upper_limit = 2000.0f;
    mock_as5600_raw_angle = 1450;

    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    cmd.packet_id = 12;
    cmd.which_cmd = opcomms_intcomm_Command_system_action_tag;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_GIMBAL_ARM;

    opcomms_intcomm_Ack ack;
    command_dispatch(&cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_EQUAL(GIMBAL_MODE_ARMED, gimbal.state.gimbal_mode);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1450.0f, gimbal.state.pan_position_setpoint);
}

void test_config_save_blocked_when_armed_and_succeeds_when_free(void) {
    gimbal.state.gimbal_mode = GIMBAL_MODE_ARMED;

    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    cmd.packet_id = 20;
    cmd.which_cmd = opcomms_intcomm_Command_system_action_tag;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_CONFIG_SAVE;

    opcomms_intcomm_Ack ack;
    command_dispatch(&cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OPERATION_FAILED, ack.code);
    TEST_ASSERT_FALSE(mock_config_saved);

    // Disarm and retry save
    gimbal.state.gimbal_mode = GIMBAL_MODE_FREE;
    gimbal.engine.pan_position_controller.Kp = 4.25f;
    gimbal.state.saved_configuration = false;

    command_dispatch(&cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_TRUE(mock_config_saved);
    TEST_ASSERT_TRUE(gimbal.state.saved_configuration);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.25f, mock_last_saved_config.pan_position_controller.Kp);
}

void test_pid_set_and_revert(void) {
    opcomms_intcomm_Command set_cmd = opcomms_intcomm_Command_init_zero;
    set_cmd.packet_id = 30;
    set_cmd.which_cmd = opcomms_intcomm_Command_pid_set_command_tag;
    set_cmd.cmd.pid_set_command.axis = opcomms_intcomm_Axis_AXIS_PAN;
    set_cmd.cmd.pid_set_command.has_params = true;
    set_cmd.cmd.pid_set_command.params.has_kp = true;
    set_cmd.cmd.pid_set_command.params.kp = 9.5f;

    opcomms_intcomm_Ack ack;
    command_dispatch(&set_cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.5f, gimbal.engine.pan_position_controller.Kp);
    TEST_ASSERT_FALSE(gimbal.state.saved_configuration);

    // Revert back to saved config (Kp = 1.0f)
    opcomms_intcomm_Command rev_cmd = opcomms_intcomm_Command_init_zero;
    rev_cmd.packet_id = 31;
    rev_cmd.which_cmd = opcomms_intcomm_Command_pid_revert_command_tag;
    rev_cmd.cmd.pid_revert_command.axis = opcomms_intcomm_Axis_AXIS_PAN;

    command_dispatch(&rev_cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, gimbal.engine.pan_position_controller.Kp);
}

void test_full_wire_roundtrip_cobs_crc_protobuf(void) {
    // 1. Host builds a StreamCommand packet
    opcomms_intcomm_IntcommPacket host_pkt = opcomms_intcomm_IntcommPacket_init_zero;
    host_pkt.which_payload = opcomms_intcomm_IntcommPacket_command_tag;
    host_pkt.payload.command.packet_id = 99;
    host_pkt.payload.command.which_cmd = opcomms_intcomm_Command_stream_command_tag;
    host_pkt.payload.command.cmd.stream_command.has_stream_rate_hz = true;
    host_pkt.payload.command.cmd.stream_command.stream_rate_hz = 100;
    host_pkt.payload.command.cmd.stream_command.has_pan_pid_stream = true;
    host_pkt.payload.command.cmd.stream_command.pan_pid_stream = true;

    uint8_t pb_buf[MAX_PB_PAYLOAD_SIZE];
    pb_ostream_t ostream = pb_ostream_from_buffer(pb_buf, sizeof(pb_buf));
    TEST_ASSERT_TRUE(pb_encode(&ostream, opcomms_intcomm_IntcommPacket_fields, &host_pkt));

    // 2. Host frames with CRC16 + COBS + 0x00 delimiter
    wire_frame_t wire;
    TEST_ASSERT_TRUE(packet_frame(pb_buf, ostream.bytes_written, &wire));

    // 3. Firmware unframes (stripping trailing 0x00) and decodes Protobuf
    raw_pb_frame_t raw_pb;
    TEST_ASSERT_TRUE(packet_unframe(wire.data, wire.len - 1, &raw_pb));

    opcomms_intcomm_Command decoded_cmd;
    TEST_ASSERT_TRUE(proto_decode_command(&raw_pb, &decoded_cmd));

    // 4. Firmware dispatches command and encodes Ack back to wire
    opcomms_intcomm_Ack ack;
    command_dispatch(&decoded_cmd, &gimbal, &ack);

    wire_frame_t ack_wire;
    TEST_ASSERT_TRUE(proto_encode_ack(&ack, &ack_wire));

    // 5. Host unframes and decodes Ack
    raw_pb_frame_t ack_raw;
    TEST_ASSERT_TRUE(packet_unframe(ack_wire.data, ack_wire.len - 1, &ack_raw));

    opcomms_intcomm_IntcommPacket rx_ack_pkt = opcomms_intcomm_IntcommPacket_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(ack_raw.data, ack_raw.len);
    TEST_ASSERT_TRUE(pb_decode(&istream, opcomms_intcomm_IntcommPacket_fields, &rx_ack_pkt));

    TEST_ASSERT_EQUAL(opcomms_intcomm_IntcommPacket_ack_tag, rx_ack_pkt.which_payload);
    TEST_ASSERT_EQUAL_UINT32(99, rx_ack_pkt.payload.ack.packet_id);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, rx_ack_pkt.payload.ack.code);
    TEST_ASSERT_EQUAL_UINT16(100, gimbal.state.stream_rate);
    TEST_ASSERT_TRUE(gimbal.state.pan_pid_stream);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_arm_fails_when_limits_unset);
    RUN_TEST(test_arm_fails_when_encoder_out_of_limits);
    RUN_TEST(test_arm_succeeds_and_locks_current_angle_as_setpoint);
    RUN_TEST(test_config_save_blocked_when_armed_and_succeeds_when_free);
    RUN_TEST(test_pid_set_and_revert);
    RUN_TEST(test_full_wire_roundtrip_cobs_crc_protobuf);
    return UNITY_END();
}
