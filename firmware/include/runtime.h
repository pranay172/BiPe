#ifndef BIPE_RUNTIME_H
#define BIPE_RUNTIME_H
#include "application.h"
#include "power_policy.h"
typedef struct {
    bipe_application application;
    bipe_power_policy policy;
    bipe_app_hal transport;
    uint32_t last_heartbeat_ms;
    bool heartbeat, disabled;
} bipe_runtime;
void bipe_runtime_init(bipe_runtime *r, const bipe_app_profile *profile,
                       bipe_app_hal hal, uint32_t evidence, uint32_t now);
bool bipe_runtime_begin(bipe_runtime *r, uint32_t now);
void bipe_runtime_tick(bipe_runtime *r, const bipe_policy_sample *sample, uint32_t now);
void bipe_runtime_stop(bipe_runtime *r, uint32_t now);
void bipe_runtime_reset(bipe_runtime *r, uint32_t now);
#endif
