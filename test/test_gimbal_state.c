#include "unity.h"
#include "gimbal/gimbal_state.h"

void setUp(void) {}
void tearDown(void) {}

void test_setup_gimbal_state_defaults(void) {
    gimbal_state_t state = {0};
    state.gimbal_mode = GIMBAL_MODE_ARMED; // Poison before setup

    setup_gimbal_state(&state);

    TEST_ASSERT_TRUE(state.saved_configuration);
    TEST_ASSERT_EQUAL(GIMBAL_MODE_FREE, state.gimbal_mode);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, state.pan_position);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, (float)GIMBAL_DEFAULT_UNSET_ROM, state.pan_position_setpoint);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, (float)GIMBAL_DEFAULT_UNSET_ROM, state.pan_lower_limit);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, (float)GIMBAL_DEFAULT_UNSET_ROM, state.pan_upper_limit);
    TEST_ASSERT_TRUE(state.streaming);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(GIMBAL_MIN_STREAM_RATE_HZ, state.stream_rate);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_setup_gimbal_state_defaults);
    return UNITY_END();
}
