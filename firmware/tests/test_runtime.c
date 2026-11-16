#include "runtime.h"
#include "policy_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { unsigned frames, disables;bool power,reject; } transport;
static bool send_frame(void *c,const bipe_frame *f) {transport *h=c;(void)f;h->frames++;return !h->reject;}
static bool disable(void *c) {transport *h=c;h->disables++;return !h->reject;}
static void power(void *c,bool permit,bool pulse) {transport *h=c;(void)pulse;h->power=permit;}
static bipe_app_profile profile(void) {
    bipe_app_profile p={.synthetic=true,.evidence=BIPE_APP_REQUIRED_EVIDENCE,
        .ranges={{0,50},{0,5},{-12.5,12.5},{-50,50},{-10,10}},
        .output_Nm_per_effort=1,.minimum_bus_V=18,.maximum_bus_V=28};
    for(unsigned j=0;j<6;j++) p.encoder_sign[j]=1;
    p.imu_to_body[0]=p.imu_to_body[4]=p.imu_to_body[8]=1;return p;
}
static void feed(bipe_runtime *r,bipe_policy_sample *s,uint32_t now) {
    bipe_application *a=&r->application;
    double g[3]={0},acc[3]={0,0,9.81};
    (void)bipe_app_imu(a,g,acc,now,now+1u);
    for(unsigned j=0;j<6;j++) {
        bipe_motor_sample m={.id=(uint8_t)(j+1),.present=true,.valid=true,
            .time_ms=now,.sequence=now+1u,.temperature=25,.position=j<4?a->qref[j]:0};
        (void)bipe_app_motor(a,&m);
    }
    a->inputs.estop_closed=a->inputs.power_good=true;a->inputs.bus_V=24;a->inputs.host_time_ms=now;
    s->time_ms=now;s->sequence++;bipe_runtime_tick(r,s,now);
}
static uint32_t active(bipe_runtime *r,transport *h,bipe_policy_sample *s) {
    memset(h,0,sizeof(*h));bipe_app_profile p=profile();
    bipe_runtime_init(r,&p,(bipe_app_hal){send_frame,power,h,disable},BIPE_POLICY_EVIDENCE_ALL,0);
    *s=(bipe_policy_sample){.dump_metal_C=40,.precharge_metal_C=40,
        .interval_peak_V=25.2,.pack_V=25.2,.valid=true,.stopping_domain_valid=true,.mechanical_energy_upper_J=4.9};
    uint32_t now=0;
    for(;now<=(uint32_t)BIPE_POLICY_COOLDOWN_MS;now+=5u) feed(r,s,now);
    now-=5u;assert(bipe_runtime_begin(r,now));
    now+=5u;feed(r,s,now);assert(h->power && h->frames==0);
    s->bus_V=24;s->ready=true;s->precharge_total_J=1.2;
    now+=5u;feed(r,s,now);assert(r->policy.state==BIPE_POLICY_RUN);
    uint32_t end=now+2110u;
    while(now<end){now++;feed(r,s,now);}
    assert(r->application.estimator_ready && r->application.supervisor.state==BIPE_READY && h->frames==0);
    r->application.driver_limits_ack_mask=0x3f;bipe_arm(&r->application.supervisor);
    now++;feed(r,s,now);
    for(unsigned i=0;i<10;i++){now++;feed(r,s,now);}
    assert(h->frames==12 && r->application.supervisor.state==BIPE_ACTIVE);
    return now;
}
int main(void) {
    bipe_runtime r;transport h={0};bipe_policy_sample s;
#if BIPE_ALLOW_SYNTHETIC_APPLICATION && BIPE_ALLOW_SYNTHETIC_POLICY
    uint32_t now=active(&r,&h,&s);unsigned before=h.frames;
    bipe_runtime_stop(&r,now);
    for(unsigned i=0;i<510;i++){now++;feed(&r,&s,now);}
    assert(h.frames>before && h.disables==1 && !h.power && r.policy.state==BIPE_POLICY_COOL);
    bipe_runtime_reset(&r,now);assert(!bipe_runtime_begin(&r,now));
    for(unsigned failure=0;failure<8;failure++) {
        now=active(&r,&h,&s);before=h.frames;
        switch(failure) {
        case 0:now+=6;bipe_runtime_tick(&r,NULL,now);break;
        case 1:s.dump_total_J=16;now++;feed(&r,&s,now);break;
        case 2:bipe_runtime_stop(&r,now);s.motoring_total_J=1;now++;feed(&r,&s,now);break;
        case 3:r.application.inputs.bus_off=true;now++;feed(&r,&s,now);break;
        case 4:now+=10;feed(&r,&s,now);break;
        case 5:r.policy.evidence=0;now++;feed(&r,&s,now);break;
        case 6:h.reject=true;now+=5;feed(&r,&s,now);break;
        case 7:r.application.profile.evidence=0;now++;feed(&r,&s,now);break;
        }
        assert(!h.power && h.disables>=1 && h.frames<=before+1u);
        assert(r.policy.state==BIPE_POLICY_FAULT || r.policy.state==BIPE_POLICY_INHIBITED);
        double debt=r.policy.dump_spent_J;bipe_runtime_reset(&r,now);
        assert(!bipe_runtime_begin(&r,now) && r.policy.dump_spent_J==debt);
    }
    now=active(&r,&h,&s);s.mechanical_energy_upper_J=5.01;now++;feed(&r,&s,now);
    assert(r.policy.state==BIPE_POLICY_STOP && r.application.supervisor.state==BIPE_STOPPING);
    puts("runtime: startup/calibration/200Hz batch, bounded stop, queue/PD disable, reset debt and eight faults passed");
#else
    bipe_app_profile p=profile();
    bipe_runtime_init(&r,&p,(bipe_app_hal){send_frame,power,&h,disable},BIPE_POLICY_EVIDENCE_ALL,0);
    assert(!bipe_runtime_begin(&r,0) && !h.power);
    p.synthetic=false;bipe_runtime_init(&r,&p,(bipe_app_hal){send_frame,power,&h,disable},BIPE_POLICY_EVIDENCE_ALL,0);
    assert(!bipe_runtime_begin(&r,0) && !h.power);
    (void)s;puts("production runtime: synthetic and incomplete real hardware blocked");
#endif
    return 0;
}
