#ifndef ACTUATOR_CORE_CRC32C_H
#define ACTUATOR_CORE_CRC32C_H

#include <stddef.h>
#include <stdint.h>

uint32_t actuator_crc32c(const uint8_t *data, size_t length);

#endif
