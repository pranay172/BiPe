/* Policy integrated through the portable runtime; real telemetry remains unqualified.
   Faults never suppress the hardware dump. No speculative cooling credit. */
#include "power_policy.h"
#include "policy_config.h"
#include "generated_config.h"
#include <math.h>
#include <string.h>
#ifndef BIPE_ALLOW_SYNTHETIC_POLICY
#define BIPE_ALLOW_SYNTHETIC_POLICY 0
#endif
#define CFG(k) BIPE_POLICY_##k
static bool authorized(const bipe_power_policy *p) {
    return p->evidence==BIPE_POLICY_EVIDENCE_ALL &&
        (p->synthetic ? BIPE_ALLOW_SYNTHETIC_POLICY : BIPE_HARDWARE_QUALIFIED);
}

static void fault(bipe_power_policy *p, enum bipe_policy_fault f, uint32_t now) {
    p->state=BIPE_POLICY_FAULT; p->fault=f; p->last_activity_ms=now; p->stable=false;
}
static bool fresh(const bipe_power_policy *p, uint32_t now) {
    return p->seen && now-p->sample.time_ms <= (uint32_t)CFG(FEEDBACK_MAX_AGE_MS);
}
static bool cold(const bipe_policy_sample *s) {
    return s->dump_metal_C<=CFG(DUMP_BEFORE_START_LIMIT_C) &&
           s->precharge_metal_C<=CFG(PRECHARGE_BEFORE_START_LIMIT_C);
}
static bool eligible(const bipe_power_policy *p, uint32_t now) {
    return authorized(p) && fresh(p,now) && p->stable && cold(&p->sample) &&
        now-p->last_activity_ms>=(uint32_t)CFG(COOLDOWN_MS) &&
        now-p->stable_since_ms>=(uint32_t)CFG(STABLE_TEMPERATURE_MS);
}
void bipe_policy_init(bipe_power_policy *p, uint32_t now, bool synthetic, uint32_t evidence) {
    memset(p,0,sizeof(*p)); p->boot_ms=now; p->last_activity_ms=now;
    p->synthetic=synthetic; p->evidence=evidence; p->qualified=authorized(p);
    p->state=p->qualified?BIPE_POLICY_COOL:BIPE_POLICY_INHIBITED;
}
void bipe_policy_request_stop(bipe_power_policy *p, uint32_t now) {
    if(p->state==BIPE_POLICY_RUN) {
        p->state=BIPE_POLICY_STOP; p->stop_ms=now;
        p->stop_work_origin_J=p->sample.motoring_total_J; p->stop_work_spent_J=0;
    }
    else if(p->state==BIPE_POLICY_START) fault(p,BIPE_POLICY_STARTUP,now);
}
bool bipe_policy_sample_update(bipe_power_policy *p, const bipe_policy_sample *s, uint32_t now) {
    if(!authorized(p)) return false;
    if(!s || !s->valid || now-s->time_ms>(uint32_t)CFG(FEEDBACK_MAX_AGE_MS) ||
       !isfinite(s->dump_metal_C) || !isfinite(s->precharge_metal_C) ||
       !isfinite(s->dump_total_J) || !isfinite(s->precharge_total_J) ||
       !isfinite(s->motoring_total_J) || s->motoring_total_J<0 ||
       !isfinite(s->mechanical_energy_upper_J) || s->mechanical_energy_upper_J<0 ||
       !isfinite(s->interval_peak_V) || !isfinite(s->pack_V) || !isfinite(s->bus_V) ||
       s->dump_metal_C<15 || s->precharge_metal_C<15 ||
       s->dump_total_J<0 || s->precharge_total_J<0 || s->interval_peak_V<0 ||
       s->pack_V<19.2 || s->pack_V>25.2 || s->bus_V<0 || s->interval_peak_V<s->bus_V) {
        fault(p,BIPE_POLICY_FEEDBACK,now); return false;
    }
    if(p->seen) {
        if(s->sequence==p->sample.sequence || s->time_ms==p->sample.time_ms) return false;
        if(s->time_ms-p->sample.time_ms>(uint32_t)CFG(FEEDBACK_MAX_AGE_MS) ||
           s->dump_total_J<p->sample.dump_total_J || s->precharge_total_J<p->sample.precharge_total_J ||
           s->motoring_total_J<p->sample.motoring_total_J) {
            fault(p,BIPE_POLICY_FEEDBACK,now); return false;
        }
    }
    bool new_heat=p->seen && (s->dump_total_J>p->sample.dump_total_J ||
                              s->precharge_total_J>p->sample.precharge_total_J ||
                              s->motoring_total_J>p->sample.motoring_total_J);
    if(p->state==BIPE_POLICY_COOL && new_heat) {
        p->last_activity_ms=now; p->stable=false;
    }
    p->sample=*s; p->seen=true;
    if(cold(s)) {
        if(!p->stable) { p->stable=true; p->stable_since_ms=now; }
    } else p->stable=false;
    if(p->cycle) {
        p->dump_spent_J=(s->dump_total_J-p->dump_origin_J)*(1+CFG(METER_RELATIVE_ERROR_REQUIREMENT))+CFG(METER_ABSOLUTE_ERROR_J_REQUIREMENT);
        p->precharge_spent_J=(s->precharge_total_J-p->precharge_origin_J)*(1+CFG(METER_RELATIVE_ERROR_REQUIREMENT))+CFG(METER_ABSOLUTE_ERROR_J_REQUIREMENT);
    }
    if(p->state==BIPE_POLICY_STOP) {
        p->stop_work_spent_J=(s->motoring_total_J-p->stop_work_origin_J)*(1+CFG(METER_RELATIVE_ERROR_REQUIREMENT))+CFG(METER_ABSOLUTE_ERROR_J_REQUIREMENT);
        if(p->stop_work_spent_J>=CFG(STOP_POSITIVE_WORK_TRIGGER_J)) {
            fault(p,BIPE_POLICY_STOP_WORK,now); return false;
        }
    }
    if(s->interval_peak_V>CFG(MAXIMUM_INTERVAL_PEAK_V)) {
        fault(p,BIPE_POLICY_PEAK,now); return false;
    }
    double dump_temperature=s->dump_metal_C+CFG(TEMPERATURE_ERROR_C_REQUIREMENT);
    double pre_temperature=s->precharge_metal_C+CFG(TEMPERATURE_ERROR_C_REQUIREMENT);
    bool active=p->state==BIPE_POLICY_START || p->state==BIPE_POLICY_RUN || p->state==BIPE_POLICY_STOP;
    if(p->cycle && active) {
        dump_temperature=fmax(dump_temperature,p->dump_origin_C+p->dump_spent_J/CFG(DUMP_LOCAL_CAPACITY_J_K));
        pre_temperature=fmax(pre_temperature,p->precharge_origin_C+p->precharge_spent_J/CFG(PRECHARGE_LOCAL_CAPACITY_J_K));
    }
    if(dump_temperature+CFG(CONTACT_K_W_REQUIREMENT)*CFG(DUMP_PEAK_EACH_W)>CFG(CASE_LIMIT_C) ||
       dump_temperature+(CFG(CONTACT_K_W_REQUIREMENT)+CFG(JUNCTION_TO_CASE_K_W))*CFG(DUMP_PEAK_EACH_W)>CFG(ELEMENT_LIMIT_C) ||
       pre_temperature+CFG(CONTACT_K_W_REQUIREMENT)*CFG(PRECHARGE_PEAK_W)>CFG(CASE_LIMIT_C) ||
       pre_temperature+(CFG(CONTACT_K_W_REQUIREMENT)+CFG(JUNCTION_TO_CASE_K_W))*CFG(PRECHARGE_PEAK_W)>CFG(ELEMENT_LIMIT_C) ||
       ((active || new_heat) && (p->dump_spent_J>CFG(CYCLE_DUMP_ENERGY_J) || p->precharge_spent_J>CFG(CYCLE_PRECHARGE_ENERGY_J)))) {
        fault(p,BIPE_POLICY_HEAT,now); return false;
    }
    if(p->state==BIPE_POLICY_RUN && (!s->stopping_domain_valid || s->mechanical_energy_upper_J>CFG(MAXIMUM_AVAILABLE_MECHANICAL_ENERGY_J) ||
       p->dump_spent_J>=CFG(NORMAL_STOP_TRIGGER_J) ||
       dump_temperature+(CFG(STOP_RESERVE_J)+.001*CFG(REACTION_UPPER_MS_REQUIREMENT)*4*CFG(DUMP_PEAK_EACH_W))/CFG(DUMP_LOCAL_CAPACITY_J_K)+CFG(CONTACT_K_W_REQUIREMENT)*CFG(DUMP_PEAK_EACH_W)>CFG(CASE_LIMIT_C)))
        bipe_policy_request_stop(p,now);
    if(p->state==BIPE_POLICY_START) {
        if(p->precharge_spent_J>=CFG(PRECHARGE_STOP_TRIGGER_J) ||
           now-p->startup_ms>=(uint32_t)CFG(STARTUP_TIMEOUT_MS)) {
            fault(p,BIPE_POLICY_STARTUP,now); return false;
        }
        if(s->ready && s->bus_V>=CFG(BUS_READY_RATIO_REQUIREMENT)*s->pack_V && s->stopping_domain_valid && s->mechanical_energy_upper_J<=CFG(MAXIMUM_AVAILABLE_MECHANICAL_ENERGY_J)) {
            p->state=BIPE_POLICY_RUN;
        }
    }
    return true;
}
void bipe_policy_tick(bipe_power_policy *p, uint32_t now) {
    p->qualified=authorized(p);
    if(!p->qualified) { p->state=BIPE_POLICY_INHIBITED; return; }
    if(p->state==BIPE_POLICY_FAULT) return;
    if(!fresh(p,now)) {
        p->stable=false;
        if(p->state==BIPE_POLICY_START || p->state==BIPE_POLICY_RUN || p->state==BIPE_POLICY_STOP)
            fault(p,BIPE_POLICY_FEEDBACK,now);
        return;
    }
    if(p->state==BIPE_POLICY_START && now-p->startup_ms>=(uint32_t)CFG(STARTUP_TIMEOUT_MS))
        fault(p,BIPE_POLICY_STARTUP,now);
    if(p->state==BIPE_POLICY_STOP && now-p->stop_ms>=(uint32_t)CFG(STOP_TIMEOUT_MS))
        fault(p,BIPE_POLICY_STOP_TIMEOUT,now);
}
bool bipe_policy_begin(bipe_power_policy *p, uint32_t now) {
    if(p->state!=BIPE_POLICY_COOL || !eligible(p,now) || !p->sample.stopping_domain_valid || p->sample.mechanical_energy_upper_J>CFG(MAXIMUM_AVAILABLE_MECHANICAL_ENERGY_J) || p->sample.bus_V>p->sample.pack_V) return false;
    p->cycle=true; p->dump_origin_J=p->sample.dump_total_J;
    p->precharge_origin_J=p->sample.precharge_total_J;
    p->dump_origin_C=p->sample.dump_metal_C+CFG(TEMPERATURE_ERROR_C_REQUIREMENT);
    p->precharge_origin_C=p->sample.precharge_metal_C+CFG(TEMPERATURE_ERROR_C_REQUIREMENT);
    p->dump_spent_J=0; p->precharge_spent_J=0;
    p->state=BIPE_POLICY_START; p->startup_ms=now; p->start_attempts++;
    return true;
}
bool bipe_policy_stopped(bipe_power_policy *p, bool disabled, uint32_t now) {
    if(p->state!=BIPE_POLICY_STOP || !disabled || !fresh(p,now)) return false;
    p->state=BIPE_POLICY_COOL; p->last_activity_ms=now; p->stable=false;
    return true;
}
void bipe_policy_reset(bipe_power_policy *p, uint32_t now) {
    /* Never clear meter baselines, debts or sample history. Ordinary reset
       starts a full cooldown; a new boot also has the same lockout. */
    if(!authorized(p)) return;
    if(p->state==BIPE_POLICY_START || p->state==BIPE_POLICY_RUN || p->state==BIPE_POLICY_STOP)
        fault(p,BIPE_POLICY_STARTUP,now);
    else { p->state=BIPE_POLICY_COOL; p->fault=BIPE_POLICY_OK;
           p->last_activity_ms=now; p->stable=false; }
}
bool bipe_policy_power_permission(const bipe_power_policy *p) {
    return authorized(p) && (p->state==BIPE_POLICY_START || p->state==BIPE_POLICY_RUN || p->state==BIPE_POLICY_STOP);
}
