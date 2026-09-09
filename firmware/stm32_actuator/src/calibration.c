#include "actuator_core/calibration.h"

#include <stddef.h>
#include <stdint.h>

static int64_t round_divide(int64_t numerator, int64_t denominator) {
    if (numerator >= 0) {
        return (numerator + (denominator / 2)) / denominator;
    }
    return -((-numerator + (denominator / 2)) / denominator);
}

static actuator_calibration_result_t validate_calibration(
    const actuator_joint_calibration_t *calibration) {
    if (calibration == NULL) {
        return ACTUATOR_CALIBRATION_NULL_ARGUMENT;
    }
    if ((calibration->positive_raw_direction != 1) &&
        (calibration->positive_raw_direction != -1)) {
        return ACTUATOR_CALIBRATION_BAD_CONFIG;
    }
    if ((calibration->minimum_raw > calibration->zero_raw) ||
        (calibration->zero_raw > calibration->maximum_raw) ||
        (calibration->maximum_raw > 4095u)) {
        return ACTUATOR_CALIBRATION_BAD_CONFIG;
    }
    return ACTUATOR_CALIBRATION_OK;
}

actuator_calibration_result_t actuator_urad_to_raw(
    const actuator_joint_calibration_t *calibration,
    int32_t position_urad,
    uint16_t *raw_position) {
    int64_t raw_delta;
    int64_t raw;
    actuator_calibration_result_t validation;

    if (raw_position == NULL) {
        return ACTUATOR_CALIBRATION_NULL_ARGUMENT;
    }
    validation = validate_calibration(calibration);
    if (validation != ACTUATOR_CALIBRATION_OK) {
        return validation;
    }

    raw_delta = round_divide(
        (int64_t)position_urad * ACTUATOR_RAW_UNITS_PER_TURN,
        ACTUATOR_TURN_URAD);
    raw = (int64_t)calibration->zero_raw +
          ((int64_t)calibration->positive_raw_direction * raw_delta);
    if ((raw < calibration->minimum_raw) ||
        (raw > calibration->maximum_raw)) {
        return ACTUATOR_CALIBRATION_LIMIT_VIOLATION;
    }

    *raw_position = (uint16_t)raw;
    return ACTUATOR_CALIBRATION_OK;
}

actuator_calibration_result_t actuator_raw_to_urad(
    const actuator_joint_calibration_t *calibration,
    uint16_t raw_position,
    int32_t *position_urad) {
    int64_t positive_raw_delta;
    int64_t urad;
    actuator_calibration_result_t validation;

    if (position_urad == NULL) {
        return ACTUATOR_CALIBRATION_NULL_ARGUMENT;
    }
    validation = validate_calibration(calibration);
    if (validation != ACTUATOR_CALIBRATION_OK) {
        return validation;
    }
    if ((raw_position < calibration->minimum_raw) ||
        (raw_position > calibration->maximum_raw)) {
        return ACTUATOR_CALIBRATION_LIMIT_VIOLATION;
    }

    positive_raw_delta =
        ((int64_t)raw_position - calibration->zero_raw) *
        calibration->positive_raw_direction;
    urad = round_divide(
        positive_raw_delta * ACTUATOR_TURN_URAD,
        ACTUATOR_RAW_UNITS_PER_TURN);
    *position_urad = (int32_t)urad;
    return ACTUATOR_CALIBRATION_OK;
}
