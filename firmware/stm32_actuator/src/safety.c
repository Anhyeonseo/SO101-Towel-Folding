#include "actuator_core/safety.h"

#include <stddef.h>

static bool heartbeat_is_fresh(const actuator_safety_t *safety, uint32_t now_ms) {
    return safety->heartbeat_seen &&
           (uint32_t)(now_ms - safety->last_heartbeat_ms) <= safety->heartbeat_timeout_ms;
}

void actuator_safety_init(actuator_safety_t *safety, uint32_t heartbeat_timeout_ms) {
    if (safety == NULL) {
        return;
    }
    safety->state = ACTUATOR_STATE_BOOT;
    safety->heartbeat_timeout_ms = heartbeat_timeout_ms;
    safety->last_heartbeat_ms = 0u;
    safety->fault_code = 0u;
    safety->heartbeat_seen = false;
    safety->hold_requested = false;
    safety->torque_disable_requested = true;
    safety->estop_asserted = false;
}

actuator_safety_result_t actuator_safety_complete_boot(actuator_safety_t *safety, bool health_ok) {
    if (safety == NULL || safety->state != ACTUATOR_STATE_BOOT) {
        return ACTUATOR_SAFETY_BAD_STATE;
    }
    if (!health_ok) {
        safety->state = ACTUATOR_STATE_FAULT;
        safety->fault_code = UINT16_C(0xFF01);
        return ACTUATOR_SAFETY_HEALTH_FAILED;
    }
    safety->state = ACTUATOR_STATE_SAFE_DISABLED;
    return ACTUATOR_SAFETY_OK;
}

void actuator_safety_on_heartbeat(actuator_safety_t *safety, uint32_t now_ms) {
    if (safety == NULL) {
        return;
    }
    safety->last_heartbeat_ms = now_ms;
    safety->heartbeat_seen = true;
}

actuator_safety_result_t actuator_safety_request_arm(
    actuator_safety_t *safety,
    bool health_ok,
    bool configuration_matches) {
    if (safety == NULL || safety->state != ACTUATOR_STATE_SAFE_DISABLED) {
        return ACTUATOR_SAFETY_BAD_STATE;
    }
    if (safety->estop_asserted) {
        return ACTUATOR_SAFETY_ESTOP_ASSERTED;
    }
    if (!health_ok) {
        return ACTUATOR_SAFETY_HEALTH_FAILED;
    }
    if (!configuration_matches) {
        return ACTUATOR_SAFETY_CONFIG_MISMATCH;
    }
    safety->state = ACTUATOR_STATE_ARMED;
    safety->hold_requested = false;
    safety->torque_disable_requested = true;
    return ACTUATOR_SAFETY_OK;
}

actuator_safety_result_t actuator_safety_request_enable(actuator_safety_t *safety, uint32_t now_ms) {
    if (safety == NULL || safety->state != ACTUATOR_STATE_ARMED) {
        return ACTUATOR_SAFETY_BAD_STATE;
    }
    if (safety->estop_asserted) {
        return ACTUATOR_SAFETY_ESTOP_ASSERTED;
    }
    if (!safety->heartbeat_seen) {
        return ACTUATOR_SAFETY_HEARTBEAT_MISSING;
    }
    if (!heartbeat_is_fresh(safety, now_ms)) {
        return ACTUATOR_SAFETY_HEARTBEAT_STALE;
    }
    safety->state = ACTUATOR_STATE_ACTIVE;
    safety->hold_requested = false;
    safety->torque_disable_requested = false;
    return ACTUATOR_SAFETY_OK;
}

actuator_safety_result_t actuator_safety_request_hold(actuator_safety_t *safety) {
    if (safety == NULL || safety->state != ACTUATOR_STATE_ACTIVE) {
        return ACTUATOR_SAFETY_BAD_STATE;
    }
    safety->state = ACTUATOR_STATE_HOLD;
    safety->hold_requested = true;
    return ACTUATOR_SAFETY_OK;
}

actuator_safety_result_t actuator_safety_request_disable(actuator_safety_t *safety) {
    if (safety == NULL || safety->state == ACTUATOR_STATE_BOOT ||
        safety->state == ACTUATOR_STATE_FAULT || safety->state == ACTUATOR_STATE_ESTOPPED) {
        return ACTUATOR_SAFETY_BAD_STATE;
    }
    safety->state = ACTUATOR_STATE_SAFE_DISABLED;
    safety->hold_requested = false;
    safety->torque_disable_requested = true;
    safety->heartbeat_seen = false;
    return ACTUATOR_SAFETY_OK;
}

void actuator_safety_report_fault(actuator_safety_t *safety, uint16_t fault_code) {
    if (safety == NULL) {
        return;
    }
    if (!safety->estop_asserted) {
        safety->state = ACTUATOR_STATE_FAULT;
    }
    safety->fault_code = fault_code;
    safety->hold_requested = false;
    safety->torque_disable_requested = true;
}

void actuator_safety_set_estop(actuator_safety_t *safety, bool asserted) {
    if (safety == NULL) {
        return;
    }
    safety->estop_asserted = asserted;
    if (asserted) {
        safety->state = ACTUATOR_STATE_ESTOPPED;
        safety->hold_requested = false;
        safety->torque_disable_requested = true;
    }
}

actuator_safety_result_t actuator_safety_clear_latched_stop(
    actuator_safety_t *safety,
    bool health_ok) {
    if (safety == NULL ||
        (safety->state != ACTUATOR_STATE_FAULT && safety->state != ACTUATOR_STATE_ESTOPPED)) {
        return ACTUATOR_SAFETY_BAD_STATE;
    }
    if (safety->estop_asserted) {
        return ACTUATOR_SAFETY_ESTOP_ASSERTED;
    }
    if (!health_ok) {
        return ACTUATOR_SAFETY_HEALTH_FAILED;
    }
    safety->state = ACTUATOR_STATE_SAFE_DISABLED;
    safety->fault_code = 0u;
    safety->heartbeat_seen = false;
    safety->hold_requested = false;
    safety->torque_disable_requested = true;
    return ACTUATOR_SAFETY_OK;
}

void actuator_safety_tick(actuator_safety_t *safety, uint32_t now_ms) {
    if (safety == NULL) {
        return;
    }
    if (safety->state == ACTUATOR_STATE_ACTIVE && !heartbeat_is_fresh(safety, now_ms)) {
        safety->state = ACTUATOR_STATE_HOLD;
        safety->hold_requested = true;
    }
}

bool actuator_safety_accepts_setpoint(const actuator_safety_t *safety) {
    return safety != NULL && safety->state == ACTUATOR_STATE_ACTIVE &&
           !safety->estop_asserted && !safety->torque_disable_requested;
}
