#include "power_policy.h"
#include "policy_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#if BIPE_ALLOW_SYNTHETIC_POLICY
static bipe_policy_sample sensor;
static uint32_t now;
static void sample(bipe_power_policy *p) {
    sensor.time_ms=now; sensor.sequence++;
    (void)bipe_policy_sample_update(p,&sensor,now);
    bipe_policy_tick(p,now);
}
static void cold_boot(bipe_power_policy *p, uint32_t start) {
    now=start;
    sensor=(bipe_policy_sample){.dump_metal_C=45,.precharge_metal_C=45,
        .interval_peak_V=25.2,.pack_V=25.2,.bus_V=0,.valid=true,.stopping_domain_valid=true};
    bipe_policy_init(p,now,true,BIPE_POLICY_EVIDENCE_ALL);
    sample(p); assert(!bipe_policy_begin(p,now));
    for(unsigned i=0;i<(unsigned)BIPE_POLICY_COOLDOWN_MS/5u;i++) { now+=5u; sample(p); }
    assert(bipe_policy_begin(p,now));
}
static void running(bipe_power_policy *p) {
    cold_boot(p,0);
    now+=5u; sensor.precharge_total_J=.4; sensor.bus_V=23;
    sensor.ready=true; sample(p);
    assert(p->state==BIPE_POLICY_RUN);
}
static void trace(FILE *f,const char *name,const bipe_power_policy *p) {
    fprintf(f,"%s,%u,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%d\n",name,now,(int)p->state,(int)p->fault,
        p->dump_spent_J,p->precharge_spent_J,sensor.motoring_total_J,p->stop_work_spent_J,sensor.interval_peak_V,
        (int)bipe_policy_power_permission(p));
}
#endif
int main(int argc,char **argv) {
    bipe_power_policy p;
    bipe_policy_init(&p,0,false,BIPE_POLICY_EVIDENCE_ALL);
    assert(p.state==BIPE_POLICY_INHIBITED && !bipe_policy_power_permission(&p));
    p.qualified=true; p.state=BIPE_POLICY_RUN;
    assert(!bipe_policy_power_permission(&p)); bipe_policy_tick(&p,0);
    assert(p.state==BIPE_POLICY_INHIBITED);
    bipe_policy_init(&p,0,true,BIPE_POLICY_EVIDENCE_ALL-1u);
    assert(p.state==BIPE_POLICY_INHIBITED);
#if BIPE_ALLOW_SYNTHETIC_POLICY
    assert(argc==2);
    FILE *f=fopen(argv[1],"w"); assert(f);
    fputs("scenario,time_ms,state,fault,dump_upper_J,precharge_upper_J,motoring_total_J,stop_work_upper_J,interval_peak_V,power_permission\n",f);
    running(&p); trace(f,"startup_success",&p);
    for(unsigned i=0;i<80;i++) {
        now+=5u; sensor.dump_total_J+=.055; sample(&p);
        trace(f,"brake_then_stop",&p);
    }
    assert(p.state==BIPE_POLICY_STOP); assert(p.dump_spent_J<6);
    for(unsigned i=0;i<50;i++) { now+=5u; sensor.dump_total_J+=.06; sample(&p); }
    assert(p.state==BIPE_POLICY_STOP && p.dump_spent_J<10.5);
    assert(!bipe_policy_stopped(&p,false,now));
    assert(bipe_policy_stopped(&p,true,now));
    double retained=p.dump_spent_J;
    bipe_policy_reset(&p,now);
    assert(p.dump_spent_J==retained && !bipe_policy_begin(&p,now));
    trace(f,"reset_retains_heat",&p);
    for(unsigned i=0;i<(unsigned)BIPE_POLICY_COOLDOWN_MS/5u;i++) { now+=5u; sample(&p); }
    assert(bipe_policy_begin(&p,now));
    trace(f,"verified_cool_restart",&p);
    cold_boot(&p,UINT32_MAX-100000u); assert(p.state==BIPE_POLICY_START);
    trace(f,"clock_wrap",&p);
    now+=5u; sensor.precharge_total_J=(BIPE_POLICY_PRECHARGE_STOP_TRIGGER_J-.1-.05)/1.05; sample(&p); assert(p.state==BIPE_POLICY_START);
    now+=5u; sensor.precharge_total_J=(BIPE_POLICY_PRECHARGE_STOP_TRIGGER_J+.01-.05)/1.05; sample(&p); assert(p.state==BIPE_POLICY_FAULT);
    assert(p.fault==BIPE_POLICY_STARTUP && !bipe_policy_power_permission(&p));
    trace(f,"shorted_precharge",&p);
    retained=p.precharge_spent_J; bipe_policy_reset(&p,now);
    assert(p.precharge_spent_J==retained && !bipe_policy_begin(&p,now));
    trace(f,"failed_start_reset",&p);
    cold_boot(&p,0);
    for(unsigned i=0;i<(unsigned)BIPE_POLICY_STARTUP_TIMEOUT_MS/5u;i++) { now+=5u; sample(&p); }
    assert(p.state==BIPE_POLICY_FAULT && p.fault==BIPE_POLICY_STARTUP);
    trace(f,"precharge_timeout",&p);
    running(&p); now+=6u; bipe_policy_tick(&p,now);
    assert(p.state==BIPE_POLICY_FAULT && p.fault==BIPE_POLICY_FEEDBACK);
    trace(f,"stale_sensor",&p);
    running(&p); now+=5u; sensor.dump_total_J=1; sample(&p);
    now+=5u; sensor.dump_total_J=.9; sample(&p);
    assert(p.state==BIPE_POLICY_FAULT && p.fault==BIPE_POLICY_FEEDBACK);
    trace(f,"meter_reset",&p);
    running(&p); now+=5u; sensor.interval_peak_V=28; sample(&p);
    assert(p.state==BIPE_POLICY_FAULT && p.fault==BIPE_POLICY_PEAK);
    trace(f,"low_energy_peak",&p);
    running(&p); now+=5u; sensor.dump_total_J=BIPE_POLICY_CYCLE_DUMP_ENERGY_J; sample(&p);
    assert(p.state==BIPE_POLICY_FAULT && p.fault==BIPE_POLICY_HEAT);
    trace(f,"energy_overload",&p);
    running(&p); now+=5u; sensor.dump_metal_C=NAN; sample(&p);
    assert(p.state==BIPE_POLICY_FAULT && p.fault==BIPE_POLICY_FEEDBACK);
    trace(f,"invalid_temperature",&p);
    running(&p); now+=5u; sensor.dump_metal_C=55; sample(&p);
    assert(p.state==BIPE_POLICY_STOP); trace(f,"hot_stop_reserve",&p);
    running(&p); now+=5u; sensor.stopping_domain_valid=false; sample(&p);
    assert(p.state==BIPE_POLICY_STOP); trace(f,"domain_exit",&p);
    for(unsigned i=0;i<(unsigned)BIPE_POLICY_STOP_TIMEOUT_MS/5u;i++) { now+=5u; sample(&p); }
    assert(p.state==BIPE_POLICY_FAULT && p.fault==BIPE_POLICY_STOP_TIMEOUT);
    trace(f,"stop_timeout",&p);
    running(&p); bipe_policy_request_stop(&p,now); assert(bipe_policy_stopped(&p,true,now));
    for(unsigned i=0;i<(unsigned)BIPE_POLICY_COOLDOWN_MS/5u-1u;i++) { now+=5u; sample(&p); }
    now+=5u; sensor.dump_total_J=.01; sample(&p);
    assert(!bipe_policy_begin(&p,now)); trace(f,"heat_during_cooldown",&p);
    running(&p); sensor.sequence++; assert(!bipe_policy_sample_update(&p,&sensor,now));
    now+=6u; bipe_policy_tick(&p,now); assert(p.state==BIPE_POLICY_FAULT);
    trace(f,"duplicate_not_fresh",&p);
    running(&p); bipe_policy_reset(&p,now);
    assert(p.state==BIPE_POLICY_FAULT && !bipe_policy_power_permission(&p));
    trace(f,"reset_while_running",&p);
    running(&p); p.evidence=0; bipe_policy_tick(&p,now);
    assert(p.state==BIPE_POLICY_INHIBITED && !bipe_policy_power_permission(&p));
    trace(f,"runtime_profile_revoked",&p);
    running(&p); bipe_policy_request_stop(&p,now);
    now+=5u; sensor.motoring_total_J=(BIPE_POLICY_STOP_POSITIVE_WORK_TRIGGER_J+.01-.05)/1.05; sample(&p);
    assert(p.state==BIPE_POLICY_FAULT && p.fault==BIPE_POLICY_STOP_WORK);
    assert(!bipe_policy_power_permission(&p)); trace(f,"positive_stop_work_cutoff",&p);
    retained=p.stop_work_spent_J; bipe_policy_reset(&p,now);
    assert(p.stop_work_spent_J==retained); trace(f,"reset_retains_stop_work",&p);
    fclose(f); puts("power policy: boundary/fault/start/stop checks passed; synthetic only");
#else
    (void)argc; (void)argv;
    bipe_policy_init(&p,0,true,BIPE_POLICY_EVIDENCE_ALL);
    assert(p.state==BIPE_POLICY_INHIBITED && !bipe_policy_power_permission(&p));
    puts("production power policy: synthetic and unqualified real profiles inhibited");
#endif
    return 0;
}
