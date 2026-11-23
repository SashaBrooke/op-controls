#include "unity.h"
#include "utils/rotary_utils.h"

void setUp(void) {}
void tearDown(void) {}

void test_rotary_error_shortest_path_no_limits(void) {
    float err = 0.0f;

    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS, calculate_rotary_error(&err, 100.0f, 300.0f, 4096.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, err);

    // Wrap forward across 0
    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS, calculate_rotary_error(&err, 3996.0f, 100.0f, 4096.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, err);

    // Wrap backward across 0
    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS, calculate_rotary_error(&err, 100.0f, 3996.0f, 4096.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -200.0f, err);
}

void test_rotary_error_standard_limits(void) {
    float err = 0.0f;

    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS,
        calculate_rotary_error__limits(&err, 1100.0f, 2900.0f, 4096.0f, 1000.0f, 3000.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1800.0f, err);

    TEST_ASSERT_EQUAL(ROTARYUTILS_UNALLOWED_REGION,
        calculate_rotary_error__limits(&err, 1500.0f, 500.0f, 4096.0f, 1000.0f, 3000.0f));
    TEST_ASSERT_EQUAL(ROTARYUTILS_UNALLOWED_REGION,
        calculate_rotary_error__limits(&err, 1500.0f, 3500.0f, 4096.0f, 1000.0f, 3000.0f));
}

void test_rotary_error_wraparound_limits(void) {
    float err = 0.0f;

    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS,
        calculate_rotary_error__limits(&err, 3900.0f, 200.0f, 4096.0f, 3500.0f, 500.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 396.0f, err);

    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS,
        calculate_rotary_error__limits(&err, 200.0f, 3900.0f, 4096.0f, 3500.0f, 500.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -396.0f, err);

    TEST_ASSERT_EQUAL(ROTARYUTILS_UNALLOWED_REGION,
        calculate_rotary_error__limits(&err, 3900.0f, 2000.0f, 4096.0f, 3500.0f, 500.0f));
}

void test_in_rotary_limits(void) {
    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS,          in_rotary_limits(500.0f,  500.0f, 1500.0f));
    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS,          in_rotary_limits(1000.0f, 500.0f, 1500.0f));
    TEST_ASSERT_EQUAL(ROTARYUTILS_UNALLOWED_REGION, in_rotary_limits(499.0f,  500.0f, 1500.0f));
    TEST_ASSERT_EQUAL(ROTARYUTILS_UNALLOWED_REGION, in_rotary_limits(1501.0f, 500.0f, 1500.0f));

    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS,          in_rotary_limits(3950.0f, 3800.0f, 200.0f));
    TEST_ASSERT_EQUAL(ROTARYUTILS_SUCCESS,          in_rotary_limits(150.0f,  3800.0f, 200.0f));
    TEST_ASSERT_EQUAL(ROTARYUTILS_UNALLOWED_REGION, in_rotary_limits(2000.0f, 3800.0f, 200.0f));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_rotary_error_shortest_path_no_limits);
    RUN_TEST(test_rotary_error_standard_limits);
    RUN_TEST(test_rotary_error_wraparound_limits);
    RUN_TEST(test_in_rotary_limits);
    return UNITY_END();
}
