#include "startup_monitor.h"
#include "policy_config.h"
#include <assert.h>
#include <stdio.h>
static void boot(bipe_startup_monitor *m,bipe_startup_sample *s) {
    *s=(bipe_startup_sample){.pack_mV=25200,.bus_mV=0,.valid=true};
    bipe_startup_init(m,0);assert(bipe_startup_update(m,s));
    assert(!m->start_ok);
    s->now_ms=(uint32_t)BIPE_POLICY_COOLDOWN_MS;s->sequence++;
    assert(bipe_startup_update(m,s));s->now_ms+=5;s->sequence++;s->armed=true;
    assert(bipe_startup_update(m,s));assert(m->start_ok);
}
int main(int argc,char **argv) {
    assert(argc==2);FILE *f=fopen(argv[1],"w");assert(f);
    fputs("case,time_ms,bus_mV,energy_upper_uJ,start_ok,fault\n",f);
    bipe_startup_monitor m;bipe_startup_sample s;
    boot(&m,&s);
    while(m.start_ok) {
        s.now_ms+=5;s.sequence++;(void)bipe_startup_update(&m,&s);
        fprintf(f,"short,%u,%u,%llu,%d,%d\n",s.now_ms,s.bus_mV,(unsigned long long)m.energy_upper_uJ,m.start_ok,m.fault);
    }
    assert(m.fault && m.energy_upper_uJ>=3200000u && m.energy_upper_uJ<3400000u);
    uint64_t debt=m.energy_upper_uJ;
    s.armed=false;s.now_ms+=5;s.sequence++;(void)bipe_startup_update(&m,&s);
    s.armed=true;s.now_ms+=5;s.sequence++;(void)bipe_startup_update(&m,&s);
    assert(!m.start_ok && m.energy_upper_uJ==debt);
    /* Independent reboot restores the full cooldown, never a retry credit. */
    bipe_startup_init(&m,s.now_ms);assert(!m.start_ok);
    boot(&m,&s);s.now_ms+=6;bipe_startup_tick(&m,s.now_ms);
    assert(m.fault && !m.start_ok);
    boot(&m,&s);s.bus_mV=24000;s.now_ms+=5;s.sequence++;
    assert(bipe_startup_update(&m,&s) && m.conducting);
    s.bus_mV=25200;
    for(unsigned i=0;i<400;i++) {s.now_ms+=5;s.sequence++;assert(bipe_startup_update(&m,&s));}
    fprintf(f,"loaded_ready,%u,%u,%llu,%d,%d\n",s.now_ms,s.bus_mV,(unsigned long long)m.energy_upper_uJ,m.start_ok,m.fault);
    /* Readiness does not switch off energy supervision: a stuck precharge
       path at a small voltage drop eventually consumes its reservation. */
    while(m.start_ok) {s.now_ms+=5;s.sequence++;(void)bipe_startup_update(&m,&s);}
    assert(m.fault && m.energy_upper_uJ>=3200000u);
    boot(&m,&s);s.bus_mV=24000;s.now_ms+=5;s.sequence++;assert(bipe_startup_update(&m,&s));
    s.bus_mV=1000;s.now_ms+=5;s.sequence++;assert(!bipe_startup_update(&m,&s));
    assert(!m.start_ok);
    boot(&m,&s);s.valid=false;s.now_ms+=5;s.sequence++;assert(!bipe_startup_update(&m,&s));
    boot(&m,&s);s.bus_mV=20000;
    for(unsigned i=0;i<100 && m.start_ok;i++){s.now_ms+=5;s.sequence++;(void)bipe_startup_update(&m,&s);}
    assert(m.fault && !m.start_ok);
    fclose(f);puts("independent startup: short, readiness, residual energy, stale, invalid, timeout and reset passed");
    return 0;
}
