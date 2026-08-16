#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define MAX_PACKET_SIZE 64

#define MAX_RX_BYTES_PER_MS MAX_PACKET_SIZE
#define MAX_TX_BYTES_PER_MS MAX_PACKET_SIZE

// Packet framing
#define SYNC_BYTE 0xAA
#define PAYLOAD_OFFSET 2 // 1 (sync) + 1 (length)

typedef struct {
    uint8_t len;
    uint8_t data[MAX_PACKET_SIZE];
} packet_t;

#endif // PACKET_H
