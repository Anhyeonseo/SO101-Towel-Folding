#include "actuator_core/crc32c.h"

uint32_t actuator_crc32c(const uint8_t *data, size_t length) {
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    size_t index;

    for (index = 0u; index < length; ++index) {
        uint32_t bit;
        crc ^= data[index];
        for (bit = 0u; bit < 8u; ++bit) {
            const uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1u) ^ (UINT32_C(0x82F63B78) & mask);
        }
    }
    return ~crc;
}
