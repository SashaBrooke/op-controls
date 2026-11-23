#include "unity.h"
#include "comms/command.h"

extern bool mock_config_saved;
extern gimbal_configuration_t mock_last_saved_config;

static gimbal_t gimbal;

void setUp(void) {
    gimbal = (gimbal_t){0};
    setup_gimbal_state(&gimbal.state);
    gimbal.engine.pan_position_controller = pid_setup(
        1.5f, 0.25f, 0.05f, 0.02f, -100.0f, 100.0f, -50.0f, 50.0f, 0.001f, 4096.0f
    );
    gimbal.config.pan_position_controller = gimbal.engine.pan_position_controller;
    gimbal.state.pan_position = 0.0f;
    mock_config_saved = false;
    mock_last_saved_config = (gimbal_configuration_t){0};
}

void tearDown(void) {}

void test_arm_and_free_lifecycle(void) {
    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    opcomms_intcomm_Ack ack;

    cmd.packet_id = 1;
    cmd.which_cmd = opcomms_intcomm_Command_system_action_tag;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_GIMBAL_ARM;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_UNSET_LIMITS, ack.code);

    gimbal.state.pan_lower_limit = 1000.0f;
    gimbal.state.pan_upper_limit = 2000.0f;
    gimbal.state.pan_position = 2500.0f;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OUT_OF_LIMITS, ack.code);

    gimbal.state.pan_position = 1450.0f;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_EQUAL(GIMBAL_MODE_ARMED, gimbal.state.gimbal_mode);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1450.0f, gimbal.state.pan_position_setpoint);

    cmd.packet_id = 2;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_GIMBAL_FREE;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_EQUAL(GIMBAL_MODE_FREE, gimbal.state.gimbal_mode);
}

void test_config_save_and_config_read_response(void) {
    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    opcomms_intcomm_Ack ack;

    gimbal.state.gimbal_mode = GIMBAL_MODE_ARMED;
    cmd.packet_id = 10;
    cmd.which_cmd = opcomms_intcomm_Command_system_action_tag;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_CONFIG_SAVE;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OPERATION_FAILED, ack.code);
    TEST_ASSERT_FALSE(mock_config_saved);

    gimbal.state.gimbal_mode = GIMBAL_MODE_FREE;
    gimbal.engine.pan_position_controller.Kp = 6.5f;
    gimbal.engine.pan_position_controller.Ki = 0.75f;
    gimbal.state.saved_configuration = false;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_TRUE(mock_config_saved);
    TEST_ASSERT_TRUE(gimbal.state.saved_configuration);

    cmd.packet_id = 11;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_CONFIG_READ;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_EQUAL(opcomms_intcomm_Ack_config_data_tag, ack.which_response_data);
    TEST_ASSERT_TRUE(ack.response_data.config_data.has_pan_pid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.5f,  ack.response_data.config_data.pan_pid.kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.75f, ack.response_data.config_data.pan_pid.ki);
}

void test_gimbal_read_response(void) {
    gimbal.state.gimbal_mode = GIMBAL_MODE_ARMED;
    gimbal.state.pan_lower_limit = 500.0f;
    gimbal.state.pan_upper_limit = 3500.0f;
    gimbal.state.pan_position = 1234.0f;
    gimbal.state.pan_position_setpoint = 1500.0f;
    gimbal.state.streaming = true;
    gimbal.state.stream_rate = 50;

    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    cmd.packet_id = 20;
    cmd.which_cmd = opcomms_intcomm_Command_system_action_tag;
    cmd.cmd.system_action = opcomms_intcomm_SystemAction_ACTION_GIMBAL_READ;

    opcomms_intcomm_Ack ack;
    command_dispatch(&cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_EQUAL(opcomms_intcomm_Ack_state_data_tag, ack.which_response_data);
    TEST_ASSERT_EQUAL(opcomms_intcomm_GimbalMode_GIMBAL_MODE_ARMED, ack.response_data.state_data.mode);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1234.0f, ack.response_data.state_data.pan_current_angle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, ack.response_data.state_data.pan_setpoint);
    TEST_ASSERT_TRUE(ack.response_data.state_data.pan_limits.is_set);
    TEST_ASSERT_TRUE(ack.response_data.state_data.streaming_global);
    TEST_ASSERT_EQUAL_UINT32(50, ack.response_data.state_data.stream_rate_hz);
}

void test_setpoint_standard_and_wraparound_limits(void) {
    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    cmd.which_cmd = opcomms_intcomm_Command_setpoint_command_tag;
    cmd.cmd.setpoint_command.axis = opcomms_intcomm_Axis_AXIS_PAN;
    opcomms_intcomm_Ack ack;

    gimbal.state.pan_lower_limit = 1000.0f;
    gimbal.state.pan_upper_limit = 2000.0f;

    cmd.cmd.setpoint_command.setpoint = 1500.0f;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);

    cmd.cmd.setpoint_command.setpoint = 2500.0f;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OUT_OF_LIMITS, ack.code);

    // Wrap-around limits
    gimbal.state.pan_lower_limit = 3800.0f;
    gimbal.state.pan_upper_limit = 300.0f;

    cmd.cmd.setpoint_command.setpoint = 100.0f;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);

    cmd.cmd.setpoint_command.setpoint = 2000.0f;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OUT_OF_LIMITS, ack.code);
}

void test_pid_validation_and_revert(void) {
    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    cmd.which_cmd = opcomms_intcomm_Command_pid_set_command_tag;
    cmd.cmd.pid_set_command.axis = opcomms_intcomm_Axis_AXIS_PAN;
    cmd.cmd.pid_set_command.has_params = true;
    opcomms_intcomm_Ack ack;

    cmd.cmd.pid_set_command.params.has_kp = true;
    cmd.cmd.pid_set_command.params.kp = -1.0f;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_INVALID_DATA, ack.code);

    cmd.cmd.pid_set_command.params.kp = 8.0f;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.0f, gimbal.engine.pan_position_controller.Kp);

    opcomms_intcomm_Command rev = opcomms_intcomm_Command_init_zero;
    rev.which_cmd = opcomms_intcomm_Command_pid_revert_command_tag;
    rev.cmd.pid_revert_command.axis = opcomms_intcomm_Axis_AXIS_PAN;
    command_dispatch(&rev, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f, gimbal.engine.pan_position_controller.Kp);
}

void test_stream_command_validation_and_toggles(void) {
    opcomms_intcomm_Command cmd = opcomms_intcomm_Command_init_zero;
    cmd.which_cmd = opcomms_intcomm_Command_stream_command_tag;
    opcomms_intcomm_Ack ack;

    cmd.cmd.stream_command.has_stream_rate_hz = true;
    cmd.cmd.stream_command.stream_rate_hz = 0;
    command_dispatch(&cmd, &gimbal, &ack);
    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_INVALID_DATA, ack.code);

    cmd.cmd.stream_command.stream_rate_hz = 100;
    cmd.cmd.stream_command.has_global_streaming = true;
    cmd.cmd.stream_command.global_streaming = true;
    cmd.cmd.stream_command.has_pan_pos_stream = true;
    cmd.cmd.stream_command.pan_pos_stream = true;
    cmd.cmd.stream_command.has_pan_pid_stream = true;
    cmd.cmd.stream_command.pan_pid_stream = true;
    command_dispatch(&cmd, &gimbal, &ack);

    TEST_ASSERT_EQUAL(opcomms_intcomm_AckCode_STATUS_OK, ack.code);
    TEST_ASSERT_EQUAL_UINT16(100, gimbal.state.stream_rate);
    TEST_ASSERT_TRUE(gimbal.state.pan_pos_stream);
    TEST_ASSERT_TRUE(gimbal.state.pan_pid_stream);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_arm_and_free_lifecycle);
    RUN_TEST(test_config_save_and_config_read_response);
    RUN_TEST(test_gimbal_read_response);
    RUN_TEST(test_setpoint_standard_and_wraparound_limits);
    RUN_TEST(test_pid_validation_and_revert);
    RUN_TEST(test_stream_command_validation_and_toggles);
    return UNITY_END();
}
