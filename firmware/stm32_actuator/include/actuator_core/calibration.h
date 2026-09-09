#ifndef ACTUATOR_CORE_CALIBRATION_H
#define ACTUATOR_CORE_CALIBRATION_H

#include <stdint.h>

#define ACTUATOR_RAW_UNITS_PER_TURN INT32_C(4096)
#define ACTUATOR_TURN_URAD INT32_C(6283185)

typedef struct {
    uint16_t zero_raw;
    uint16_t minimum_raw;
    uint16_t maximum_raw;
    int8_t positive_raw_direction;
} actuator_joint_calibration_t;

typedef enum {
    ACTUATOR_CALIBRATION_OK = 0,
    ACTUATOR_CALIBRATION_NULL_ARGUMENT,
    ACTUATOR_CALIBRATION_BAD_CONFIG,
    ACTUATOR_CALIBRATION_LIMIT_VIOLATION
} actuator_calibration_result_t;

actuator_calibration_result_t actuator_urad_to_raw(
    const actuator_joint_calibration_t *calibration,
    int32_t position_urad,
    uint16_t *raw_position);

actuator_calibration_result_t actuator_raw_to_urad(
    const actuator_joint_calibration_t *calibration,
    uint16_t raw_position,
    int32_t *position_urad);

#endif
