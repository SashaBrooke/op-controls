#include <string.h>
#include "comms/packet.h"

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

static size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output) {
    size_t read_idx = 0, write_idx = 1, code_idx = 0;
    uint8_t code = 1;

    while (read_idx < length) {
        if (input[read_idx] == 0) {
            output[code_idx] = code;
            code = 1;
            code_idx = write_idx++;
            read_idx++;
        } else {
            output[write_idx++] = input[read_idx++];
            if (++code == 0xFF) {
                output[code_idx] = code;
                code = 1;
                code_idx = write_idx++;
            }
        }
    }
    output[code_idx] = code;
    return write_idx;
}

static size_t cobs_decode(const uint8_t *input, size_t length, uint8_t *output) {
    size_t read_idx = 0, write_idx = 0;
    while (read_idx < length) {
        uint8_t code = input[read_idx];
        if (code == 0 || (read_idx + code > length && code != 1)) return 0;
        read_idx++;
        for (uint8_t i = 1; i < code; i++) {
            if (read_idx >= length) return 0;
            output[write_idx++] = input[read_idx++];
        }
        if (code < 0xFF && read_idx < length) {
            output[write_idx++] = 0x00;
        }
    }
    return write_idx;
}

bool packet_unframe(const uint8_t *cobs_data, size_t cobs_len, raw_pb_frame_t *out_pb) {
    uint8_t decoded[MAX_PB_PAYLOAD_SIZE + 2];
    size_t dec_len = cobs_decode(cobs_data, cobs_len, decoded);
    if (dec_len < 2) return false;

    size_t payload_len = dec_len - 2;
    if (payload_len > MAX_PB_PAYLOAD_SIZE) return false;

    uint16_t rx_crc = ((uint16_t)decoded[payload_len] << 8) | decoded[payload_len + 1];
    if (crc16_ccitt(decoded, payload_len) != rx_crc) {
        return false;
    }

    memcpy(out_pb->data, decoded, payload_len);
    out_pb->len = (uint16_t)payload_len;
    return true;
}

bool packet_frame(const uint8_t *pb_data, size_t pb_len, wire_frame_t *out_wire) {
    if (pb_len > MAX_PB_PAYLOAD_SIZE) return false;

    uint8_t buf_with_crc[MAX_PB_PAYLOAD_SIZE + 2];
    memcpy(buf_with_crc, pb_data, pb_len);

    uint16_t crc = crc16_ccitt(pb_data, pb_len);
    buf_with_crc[pb_len]     = (crc >> 8) & 0xFF;
    buf_with_crc[pb_len + 1] = crc & 0xFF;

    size_t encoded_len = cobs_encode(buf_with_crc, pb_len + 2, out_wire->data);
    out_wire->data[encoded_len++] = 0x00;
    out_wire->len = (uint16_t)encoded_len;
    return true;
}
