#ifndef ACTUATOR_CORE_COBS_H
#define ACTUATOR_CORE_COBS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool actuator_cobs_encode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

bool actuator_cobs_decode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

#endif
