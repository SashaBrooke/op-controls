#include "unity.h"
#include "controls/pid.h"

void setUp(void) {}
void tearDown(void) {}

void test_pid_proportional_and_output_clamping(void) {
    pid_controller_t pid = pid_setup(
        2.0f, 0.0f, 0.0f, 0.0f,
        -100.0f, 100.0f, -50.0f, 50.0f,
        0.01f, 4096.0f
    );

    float out = pid_update(&pid, 120.0f, 100.0f, 0.0f, 4095.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, pid.proportional);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, out);

    // Output clamps at +100
    out = pid_update(&pid, 300.0f, 100.0f, 0.0f, 4095.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, out);
}

void test_pid_integrator_clamping_and_reset(void) {
    pid_controller_t pid = pid_setup(
        0.0f, 10.0f, 0.0f, 0.0f,
        -100.0f, 100.0f, -25.0f, 25.0f,
        0.1f, 4096.0f
    );

    for (int i = 0; i < 20; i++) {
        pid_update(&pid, 100.0f, 0.0f, 0.0f, 4095.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, pid.integrator);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, pid.output);

    pid_reset(&pid);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.proportional);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.integrator);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.differentiator);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.output);
}

void test_pid_normalise_output(void) {
    pid_controller_t pid = pid_setup(
        1.0f, 0.0f, 0.0f, 0.0f,
        -100.0f, 100.0f, -50.0f, 50.0f,
        0.01f, 4096.0f
    );
    pid.output = 50.0f;

    float norm = pid_normalise_output(&pid, 0.0f, 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 750.0f, norm);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_proportional_and_output_clamping);
    RUN_TEST(test_pid_integrator_clamping_and_reset);
    RUN_TEST(test_pid_normalise_output);
    return UNITY_END();
}
