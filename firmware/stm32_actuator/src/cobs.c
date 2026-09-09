#include "actuator_core/cobs.h"

bool actuator_cobs_encode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length) {
    size_t read_index = 0u;
    size_t write_index = 1u;
    size_t code_index = 0u;
    uint8_t code = 1u;

    if (output == NULL || output_length == NULL || output_capacity == 0u) {
        return false;
    }
    if (input_length > 0u && input == NULL) {
        return false;
    }

    while (read_index < input_length) {
        if (input[read_index] == 0u) {
            if (code_index >= output_capacity) {
                return false;
            }
            output[code_index] = code;
            code = 1u;
            code_index = write_index;
            ++write_index;
            ++read_index;
        } else {
            if (write_index >= output_capacity) {
                return false;
            }
            output[write_index] = input[read_index];
            ++write_index;
            ++read_index;
            ++code;
            if (code == 0xFFu) {
                if (code_index >= output_capacity) {
                    return false;
                }
                output[code_index] = code;
                code = 1u;
                code_index = write_index;
                ++write_index;
            }
        }
    }

    if (code_index >= output_capacity) {
        return false;
    }
    output[code_index] = code;
    *output_length = write_index;
    return true;
}

bool actuator_cobs_decode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length) {
    size_t read_index = 0u;
    size_t write_index = 0u;

    if (input == NULL || output == NULL || output_length == NULL || input_length == 0u) {
        return false;
    }

    while (read_index < input_length) {
        uint8_t code = input[read_index];
        size_t copy_count;
        size_t offset;

        if (code == 0u) {
            return false;
        }
        ++read_index;
        copy_count = (size_t)code - 1u;
        if (copy_count > input_length - read_index || copy_count > output_capacity - write_index) {
            return false;
        }
        for (offset = 0u; offset < copy_count; ++offset) {
            output[write_index++] = input[read_index++];
        }
        if (code != 0xFFu && read_index < input_length) {
            if (write_index >= output_capacity) {
                return false;
            }
            output[write_index++] = 0u;
        }
    }

    *output_length = write_index;
    return true;
}
