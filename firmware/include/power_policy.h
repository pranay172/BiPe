#ifndef BIPE_POWER_POLICY_H
#define BIPE_POWER_POLICY_H
#include <stdbool.h>
#include <stdint.h>

enum bipe_policy_state { BIPE_POLICY_INHIBITED, BIPE_POLICY_COOL,
    BIPE_POLICY_START, BIPE_POLICY_RUN, BIPE_POLICY_STOP, BIPE_POLICY_FAULT };
enum bipe_policy_fault { BIPE_POLICY_OK, BIPE_POLICY_FEEDBACK,
    BIPE_POLICY_PEAK, BIPE_POLICY_HEAT, BIPE_POLICY_STARTUP, BIPE_POLICY_STOP_TIMEOUT,
    BIPE_POLICY_STOP_WORK };
#define BIPE_POLICY_EVIDENCE_ALL 0xffu
/* Eight independent gates: exact motor voltage; thermal interface/environment;
   two sensor error/lag bounds; three monotonic energy meters; interval peak
   latch; startup cutoff; bounded stopping domain; real timing/transport. */
typedef struct {
    uint32_t time_ms, sequence;
    double dump_metal_C, precharge_metal_C;
    double dump_total_J, precharge_total_J, motoring_total_J;
    double interval_peak_V, pack_V, bus_V;
    double mechanical_energy_upper_J; /* KE + positive PE to ground, incl uncertainty */
    bool valid, ready, stopping_domain_valid;
} bipe_policy_sample;
typedef struct {
    enum bipe_policy_state state;
    enum bipe_policy_fault fault;
    bipe_policy_sample sample;
    uint32_t boot_ms, last_activity_ms, stable_since_ms, startup_ms, stop_ms;
    uint32_t evidence;
    double dump_origin_J, precharge_origin_J, dump_origin_C, precharge_origin_C;
    double dump_spent_J, precharge_spent_J;
    double stop_work_origin_J, stop_work_spent_J;
    bool qualified, synthetic, seen, stable, cycle;
    unsigned start_attempts;
} bipe_power_policy;
void bipe_policy_init(bipe_power_policy *p, uint32_t now_ms, bool synthetic, uint32_t evidence);
bool bipe_policy_sample_update(bipe_power_policy *p, const bipe_policy_sample *s, uint32_t now_ms);
void bipe_policy_tick(bipe_power_policy *p, uint32_t now_ms);
bool bipe_policy_begin(bipe_power_policy *p, uint32_t now_ms);
void bipe_policy_request_stop(bipe_power_policy *p, uint32_t now_ms);
bool bipe_policy_stopped(bipe_power_policy *p, bool disabled, uint32_t now_ms);
void bipe_policy_reset(bipe_power_policy *p, uint32_t now_ms);
bool bipe_policy_power_permission(const bipe_power_policy *p);
#endif
