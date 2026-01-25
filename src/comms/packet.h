#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define MAX_PACKET_SIZE 64

#define MAX_RX_BYTES_PER_MS MAX_PACKET_SIZE
#define MAX_TX_BYTES_PER_MS MAX_PACKET_SIZE

// Packet framing
#define PACKET_MARKER 0xAA
#define PACKET_MARKER_SIZE 1 // Bytes

typedef struct {
    uint8_t len;
    uint8_t data[MAX_PACKET_SIZE];
} packet_t;

#endif // PACKET_H
