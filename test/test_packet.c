#include <string.h>
#include "unity.h"
#include "comms/packet.h"

void setUp(void) {}
void tearDown(void) {}

void test_packet_frame_and_unframe_with_embedded_zeros(void) {
    const uint8_t payload[] = {0x00, 0x11, 0x00, 0x22, 0x33, 0x00, 0xFF};
    wire_frame_t wire = {0};

    TEST_ASSERT_TRUE(packet_frame(payload, sizeof(payload), &wire));
    TEST_ASSERT_GREATER_THAN(sizeof(payload), wire.len);

    // Must end with 0x00 delimiter and contain zero 0x00 bytes before the final byte
    TEST_ASSERT_EQUAL_UINT8(0x00, wire.data[wire.len - 1]);
    for (size_t i = 0; i < wire.len - 1; i++) {
        TEST_ASSERT_NOT_EQUAL_UINT8(0x00, wire.data[i]);
    }

    // Unframe (passing payload up to but excluding trailing 0x00)
    raw_pb_frame_t out = {0};
    TEST_ASSERT_TRUE(packet_unframe(wire.data, wire.len - 1, &out));
    TEST_ASSERT_EQUAL_UINT(sizeof(payload), out.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, out.data, sizeof(payload));
}

void test_packet_unframe_rejects_corrupted_crc(void) {
    const uint8_t payload[] = {0xAA, 0xBB, 0xCC, 0xDD};
    wire_frame_t wire = {0};

    TEST_ASSERT_TRUE(packet_frame(payload, sizeof(payload), &wire));

    // Flip a byte while keeping it non-zero so COBS structure stays intact
    wire.data[1] ^= 0x55;
    if (wire.data[1] == 0x00) wire.data[1] = 0x01;

    raw_pb_frame_t out = {0};
    TEST_ASSERT_FALSE(packet_unframe(wire.data, wire.len - 1, &out));
}

void test_packet_rejects_oversized_or_truncated_inputs(void) {
    uint8_t oversized[MAX_PB_PAYLOAD_SIZE + 1] = {0};
    wire_frame_t wire = {0};
    TEST_ASSERT_FALSE(packet_frame(oversized, sizeof(oversized), &wire));

    uint8_t too_short[] = {0x01, 0x02};
    raw_pb_frame_t out = {0};
    TEST_ASSERT_FALSE(packet_unframe(too_short, sizeof(too_short), &out));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_packet_frame_and_unframe_with_embedded_zeros);
    RUN_TEST(test_packet_unframe_rejects_corrupted_crc);
    RUN_TEST(test_packet_rejects_oversized_or_truncated_inputs);
    return UNITY_END();
}
