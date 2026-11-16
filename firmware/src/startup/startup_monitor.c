/* No dependence on the primary processor. Outputs default low; fault output
   must latch FAULT_N low on PA1, START_OK on PA7, and WDT must reset ≤10 ms.
   Native ADC/UPDI/WDT glue is a physical commissioning gate, not simulated IO. */
#include "startup_monitor.h"
#include "policy_config.h"
#include <string.h>
static void fail(bipe_startup_monitor *m,uint32_t now) {
    m->start_ok=false;m->fault=true;m->attempting=false;m->conducting=false;
    m->cooldown_ms=now;m->low_seen=false;
}
void bipe_startup_init(bipe_startup_monitor *m,uint32_t now) {
    memset(m,0,sizeof(*m));m->cooldown_ms=now;
}
void bipe_startup_tick(bipe_startup_monitor *m,uint32_t now) {
    if((m->attempting || m->conducting) && (!m->seen || now-m->last_ms>5u)) fail(m,now);
    if(m->attempting && now-m->started_ms>=(uint32_t)BIPE_POLICY_STARTUP_TIMEOUT_MS) fail(m,now);
}
bool bipe_startup_update(bipe_startup_monitor *m,const bipe_startup_sample *s) {
    if(!s) return false;
    if(!s->valid || s->pack_mV<19200u || s->pack_mV>25200u || s->bus_mV>27000u) {
        fail(m,s->now_ms);return false;
    }
    if(m->seen && (s->sequence==m->last_sequence || s->now_ms==m->last_ms)) return false;
    uint32_t dt=m->seen?s->now_ms-m->last_ms:0;
    if((m->attempting || m->conducting) && dt>5u) { fail(m,s->now_ms);return false; }
    uint32_t delta=s->pack_mV+500u>s->bus_mV?s->pack_mV+500u-s->bus_mV:0;
    if(m->attempting || m->conducting) {
        uint64_t d=delta>m->previous_delta_mV?delta:m->previous_delta_mV;
        m->energy_upper_uJ+=(d*d*dt+19999u)/20000u;
        if(m->energy_upper_uJ>=3200000u) { fail(m,s->now_ms);return false; }
    }
    m->seen=true;m->last_ms=s->now_ms;m->last_sequence=s->sequence;m->previous_delta_mV=delta;
    if(!s->armed) {
        if(m->attempting || m->conducting) fail(m,s->now_ms);
        m->low_seen=true;m->start_ok=false;
        return true;
    }
    if(!m->attempting && !m->conducting && m->low_seen &&
       s->now_ms-m->cooldown_ms>=(uint32_t)BIPE_POLICY_COOLDOWN_MS) {
        m->fault=false;m->attempting=true;m->start_ok=true;m->started_ms=s->now_ms;
        m->energy_upper_uJ=0;
    }
    if(m->attempting && s->bus_mV>=500u && (uint64_t)(s->bus_mV-250u)*10u>=(uint64_t)(s->pack_mV+250u)*9u) {
        m->conducting=true;m->attempting=false;
    }
    if(m->conducting && ((uint64_t)(s->bus_mV+250u)*10u<(uint64_t)(s->pack_mV-250u)*9u)) {
        fail(m,s->now_ms);return false;
    }
    bipe_startup_tick(m,s->now_ms);return !m->fault;
}
