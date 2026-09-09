#ifndef ACTUATOR_CORE_SAFETY_H
#define ACTUATOR_CORE_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ACTUATOR_STATE_BOOT = 0,
    ACTUATOR_STATE_SAFE_DISABLED,
    ACTUATOR_STATE_ARMED,
    ACTUATOR_STATE_ACTIVE,
    ACTUATOR_STATE_HOLD,
    ACTUATOR_STATE_FAULT,
    ACTUATOR_STATE_ESTOPPED
} actuator_state_t;

typedef enum {
    ACTUATOR_SAFETY_OK = 0,
    ACTUATOR_SAFETY_BAD_STATE,
    ACTUATOR_SAFETY_HEALTH_FAILED,
    ACTUATOR_SAFETY_CONFIG_MISMATCH,
    ACTUATOR_SAFETY_ESTOP_ASSERTED,
    ACTUATOR_SAFETY_HEARTBEAT_MISSING,
    ACTUATOR_SAFETY_HEARTBEAT_STALE
} actuator_safety_result_t;

typedef struct {
    actuator_state_t state;
    uint32_t heartbeat_timeout_ms;
    uint32_t last_heartbeat_ms;
    uint16_t fault_code;
    bool heartbeat_seen;
    bool hold_requested;
    bool torque_disable_requested;
    bool estop_asserted;
} actuator_safety_t;

void actuator_safety_init(actuator_safety_t *safety, uint32_t heartbeat_timeout_ms);
actuator_safety_result_t actuator_safety_complete_boot(actuator_safety_t *safety, bool health_ok);
void actuator_safety_on_heartbeat(actuator_safety_t *safety, uint32_t now_ms);
actuator_safety_result_t actuator_safety_request_arm(
    actuator_safety_t *safety,
    bool health_ok,
    bool configuration_matches);
actuator_safety_result_t actuator_safety_request_enable(actuator_safety_t *safety, uint32_t now_ms);
actuator_safety_result_t actuator_safety_request_hold(actuator_safety_t *safety);
actuator_safety_result_t actuator_safety_request_disable(actuator_safety_t *safety);
void actuator_safety_report_fault(actuator_safety_t *safety, uint16_t fault_code);
void actuator_safety_set_estop(actuator_safety_t *safety, bool asserted);
actuator_safety_result_t actuator_safety_clear_latched_stop(
    actuator_safety_t *safety,
    bool health_ok);
void actuator_safety_tick(actuator_safety_t *safety, uint32_t now_ms);
bool actuator_safety_accepts_setpoint(const actuator_safety_t *safety);

#endif
