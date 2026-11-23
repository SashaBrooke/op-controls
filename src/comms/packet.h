#ifndef COMMS_PACKET_H
#define COMMS_PACKET_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define MAX_PB_PAYLOAD_SIZE  128
#define MAX_WIRE_FRAME_SIZE  (MAX_PB_PAYLOAD_SIZE + 8)

// Passed from comm_rx_task -> rxQueue -> comm_process_task
typedef struct {
    uint8_t  data[MAX_PB_PAYLOAD_SIZE];
    uint16_t len;
} raw_pb_frame_t;

// Passed from comm_process_task / stream_task -> txQueue -> comm_tx_task
typedef struct {
    uint8_t  data[MAX_WIRE_FRAME_SIZE];
    uint16_t len;
} wire_frame_t;

// Unframes COBS + verifies CRC16 -> outputs raw Protobuf payload
bool packet_unframe(const uint8_t *cobs_data, size_t cobs_len, raw_pb_frame_t *out_pb);

// Appends CRC16 + COBS encodes + appends 0x00 delimiter -> outputs wire frame
bool packet_frame(const uint8_t *pb_data, size_t pb_len, wire_frame_t *out_wire);

#endif // COMMS_PACKET_H
