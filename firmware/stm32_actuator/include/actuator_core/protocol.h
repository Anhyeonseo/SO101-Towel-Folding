#ifndef ACTUATOR_CORE_PROTOCOL_H
#define ACTUATOR_CORE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "actuator_core/message_ids.h"

#define ACTUATOR_PROTOCOL_MAGIC UINT16_C(0xA55A)
#define ACTUATOR_PROTOCOL_MAX_PAYLOAD 512u
#define ACTUATOR_PROTOCOL_HEADER_SIZE 16u
#define ACTUATOR_PROTOCOL_CRC_SIZE 4u
#define ACTUATOR_PROTOCOL_MAX_DECODED_SIZE \
    (ACTUATOR_PROTOCOL_HEADER_SIZE + ACTUATOR_PROTOCOL_MAX_PAYLOAD + ACTUATOR_PROTOCOL_CRC_SIZE)
#define ACTUATOR_PROTOCOL_MAX_ENCODED_SIZE \
    (ACTUATOR_PROTOCOL_MAX_DECODED_SIZE + (ACTUATOR_PROTOCOL_MAX_DECODED_SIZE / 254u) + 2u)

typedef struct {
    uint8_t message_type;
    uint16_t flags;
    uint32_t sequence;
    uint32_t sender_time_ms;
    uint16_t payload_length;
    uint8_t payload[ACTUATOR_PROTOCOL_MAX_PAYLOAD];
} actuator_frame_t;

typedef enum {
    ACTUATOR_PROTOCOL_OK = 0,
    ACTUATOR_PROTOCOL_NO_FRAME,
    ACTUATOR_PROTOCOL_NULL_ARGUMENT,
    ACTUATOR_PROTOCOL_OUTPUT_TOO_SMALL,
    ACTUATOR_PROTOCOL_OVERFLOW,
    ACTUATOR_PROTOCOL_COBS_ERROR,
    ACTUATOR_PROTOCOL_FRAME_TOO_SHORT,
    ACTUATOR_PROTOCOL_BAD_MAGIC,
    ACTUATOR_PROTOCOL_BAD_VERSION,
    ACTUATOR_PROTOCOL_UNKNOWN_MESSAGE,
    ACTUATOR_PROTOCOL_BAD_LENGTH,
    ACTUATOR_PROTOCOL_BAD_CRC
} actuator_protocol_result_t;

typedef struct {
    uint8_t encoded[ACTUATOR_PROTOCOL_MAX_ENCODED_SIZE - 1u];
    size_t length;
    bool dropping_overflowed_frame;
} actuator_stream_parser_t;

bool actuator_protocol_message_is_known(uint8_t message_type);

actuator_protocol_result_t actuator_frame_encode(
    const actuator_frame_t *frame,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

actuator_protocol_result_t actuator_frame_decode(
    const uint8_t *encoded,
    size_t encoded_length,
    actuator_frame_t *frame);

void actuator_stream_parser_init(actuator_stream_parser_t *parser);

actuator_protocol_result_t actuator_stream_parser_push(
    actuator_stream_parser_t *parser,
    uint8_t byte,
    actuator_frame_t *frame);

#endif
