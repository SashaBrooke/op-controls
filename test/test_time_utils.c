#include "unity.h"
#include "utils/time_utils.h"

void setUp(void) {}
void tearDown(void) {}

void test_freq_to_period(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.001f, FREQ2PERIOD(1000));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f,  FREQ2PERIOD(20));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f,   FREQ2PERIOD(1));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f,   FREQ2PERIOD(0)); // Zero-division guard
}

void test_secs_to_msecs(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,    SECS2MSECS(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, SECS2MSECS(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2500.0f, SECS2MSECS(2.5f));
}

void test_secs_to_usecs(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f,       SECS2USECS(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000000.0f, SECS2USECS(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 500.0f,     SECS2USECS(0.0005f));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_freq_to_period);
    RUN_TEST(test_secs_to_msecs);
    RUN_TEST(test_secs_to_usecs);
    return UNITY_END();
}
