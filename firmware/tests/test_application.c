#include "application.h"
#include "generated_config.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct { unsigned frames, edges, disables; bool permission, level, reject; } fake_hal;
static bool disable(void *ctx) { fake_hal *h=ctx; h->disables++; return !h->reject; }
static bool send_frame(void *ctx,const bipe_frame *f) {
    fake_hal *h=ctx;
    if(h->reject) return false;
    assert(f->extended && f->dlc==8 && (f->id>>8)==8);
    h->frames++; return true;
}
static void power(void *ctx,bool permission,bool level) {
    fake_hal *h=ctx;
    if(permission && level!=h->level) h->edges++;
    h->permission=permission; h->level=level;
}
static bipe_app_profile synthetic(void) {
    bipe_app_profile p={0}; p.synthetic=true; p.evidence=BIPE_APP_REQUIRED_EVIDENCE;
    p.ranges=(bipe_ranges){{0,50},{0,5},{-12.5,12.5},{-50,50},{-10,10}};
    p.output_Nm_per_effort=1; p.minimum_bus_V=18; p.maximum_bus_V=28;
    for(unsigned j=0;j<6;j++) p.encoder_sign[j]=1;
    p.imu_to_body[0]=p.imu_to_body[4]=p.imu_to_body[8]=1;
    return p;
}
static void feed(bipe_application *a,uint32_t now) {
    double g[3]={0},accel[3]={0,0,9.81};
    (void)bipe_app_imu(a,g,accel,now,now+1u);
    for(unsigned j=0;j<6;j++) {
        bipe_motor_sample m={0}; m.id=(uint8_t)(j+1); m.present=m.valid=true;
        m.time_ms=now; m.sequence=now+1u; m.temperature=25;
        m.position=j<4?a->qref[j]:0;
        (void)bipe_app_motor(a,&m);
    }
    a->inputs.estop_closed=a->inputs.power_good=true; a->inputs.bus_V=24;
    a->inputs.host_time_ms=now;
}
static void active(bipe_application *a,fake_hal *h) {
    bipe_app_profile p=synthetic(); memset(h,0,sizeof(*h));
    bipe_app_init(a,&p,(bipe_app_hal){send_frame,power,h,disable}); bipe_start(&a->supervisor);
    for(uint32_t t=0;t<=2100;t++) { feed(a,t); bipe_app_tick(a,t); }
    assert(a->supervisor.state==BIPE_READY && a->estimator_ready && h->frames==0);
    a->driver_limits_ack_mask=0x3f; bipe_arm(&a->supervisor);
    feed(a,2101); bipe_app_tick(a,2101);
    for(uint32_t t=2102;t<=2110;t++) { feed(a,t); bipe_app_tick(a,t); }
    assert(a->supervisor.state==BIPE_ACTIVE && h->frames==12 && h->edges>100);
    for(unsigned j=0;j<6;j++) assert(fabs(a->last_effort[j])<=(j<4?2.5:.6));
}
int main(void) {
    fake_hal h={0}; bipe_application a; bipe_app_profile p=synthetic();
    bipe_app_hal hal={send_frame,power,&h,disable};
#if BIPE_ALLOW_SYNTHETIC_APPLICATION
    active(&a,&h);
    unsigned initial=h.frames;
    /* Startup never emits motor frames; six-frame batches run at 200Hz. */
    for(uint32_t t=2111;t<=2160;t++) { feed(&a,t); bipe_app_tick(&a,t); }
    assert(h.frames-initial==60);
    for(uint32_t t=2161;t<=3000;t++) {
        feed(&a,t); a.inputs.host_time_ms=2160; bipe_app_tick(&a,t);
    }
    assert(a.supervisor.state==BIPE_READY && !a.supervisor.torque_enabled && h.disables==1);
    initial=h.frames;
    for(uint32_t t=3001;t<=3030;t++) { feed(&a,t); bipe_app_tick(&a,t); }
    assert(h.frames==initial); /* Restored host does not automatically arm. */
    for(unsigned failure=0;failure<8;failure++) {
        active(&a,&h); unsigned before=h.frames; feed(&a,2111);
        switch(failure) {
        case 0: a.inputs.bus_off=true; break;
        case 1: a.inputs.imu_time_ms=2080; break;
        case 2: a.inputs.motor[2].time_ms=2080; break;
        case 3: a.inputs.estop_closed=false; break;
        case 4: a.profile.evidence=0; break;
        case 5: a.driver_limits_ack_mask=0; break;
        case 6: a.inputs.bus_V=NAN; break;
        case 7: a.profile.encoder_sign[0]=0; break;
        }
        bipe_app_tick(&a,failure==5?2115:2111);
        assert(a.supervisor.state==BIPE_FAULT && !h.permission && !h.level && h.frames==before);
        feed(&a,2116); bipe_app_tick(&a,2116); assert(a.supervisor.state==BIPE_FAULT);
        bipe_app_reset(&a); assert(a.supervisor.state==BIPE_OFF && !a.inputs.imu_present && !h.permission);
    }
    active(&a,&h); h.reject=true; feed(&a,2115); bipe_app_tick(&a,2115);
    assert(a.supervisor.fault==BIPE_BUS_OFF && !h.permission);
    active(&a,&h); feed(&a,2130); bipe_app_tick(&a,2130);
    assert(a.supervisor.fault==BIPE_DEADLINE && !h.permission);
    active(&a,&h); assert(!bipe_app_command(&a,NAN,0,a.qref,a.qdref,2111));
    assert(!h.permission && a.supervisor.fault==BIPE_HOST);
    active(&a,&h);
    bipe_motor_sample duplicate=a.inputs.motor[0];
    assert(!bipe_app_motor(&a,&duplicate)); assert(a.supervisor.state==BIPE_ACTIVE);
    duplicate.time_ms=2111; duplicate.sequence++; duplicate.position+=1;
    assert(!bipe_app_motor(&a,&duplicate)); assert(a.supervisor.fault==BIPE_ENCODER);
    active(&a,&h); bipe_app_reset(&a); assert(h.disables==1 && !h.permission);
    bipe_start(&a.supervisor); feed(&a,0); bipe_app_tick(&a,0);
    assert(a.supervisor.state!=BIPE_ACTIVE && h.frames==12);
    /* A complete numeric profile still cannot admit real hardware in this build. */
    p=synthetic(); p.synthetic=false; memset(&h,0,sizeof(h)); bipe_app_init(&a,&p,hal);
    bipe_start(&a.supervisor); feed(&a,0); bipe_app_tick(&a,0);
    assert(a.supervisor.fault==BIPE_PROFILE && h.frames==0 && !h.permission);
    p=synthetic(); p.imu_to_body[8]=-1; bipe_app_init(&a,&p,hal); assert(!a.profile_valid);
    p=synthetic(); p.output_Nm_per_effort=0; bipe_app_init(&a,&p,hal); assert(!a.profile_valid);
    p=synthetic(); p.ranges.effort=(bipe_range){-DBL_MAX,DBL_MAX};
    bipe_app_init(&a,&p,hal); assert(!a.profile_valid);
    /* Wrong moving calibration cannot reach READY or emit a command. */
    p=synthetic(); memset(&h,0,sizeof(h)); bipe_app_init(&a,&p,hal); bipe_start(&a.supervisor);
    for(uint32_t t=0;t<=2200;t++) { feed(&a,t); a.inputs.motor[0].speed=1; bipe_app_tick(&a,t); }
    assert(a.supervisor.state==BIPE_CALIBRATING && !a.estimator_ready && h.frames==0);
    /* Nontrivial raw-to-body axis and encoder mappings are explicit. */
    p=synthetic(); p.encoder_sign[0]=-1; p.encoder_zero[0]=.2;
    p.imu_to_body[0]=p.imu_to_body[4]=-1;
    bipe_app_init(&a,&p,hal);
    bipe_motor_sample mapped={.present=true,.valid=true,.time_ms=1,.sequence=1,.position=.1,.speed=.4,.id=1,.temperature=25};
    assert(bipe_app_motor(&a,&mapped)); assert(fabs(a.inputs.motor[0].position-.1)<1e-12 && a.inputs.motor[0].speed==-.4);
    double g[3]={.1,.2,.3},ac[3]={1,2,9.81};
    assert(bipe_app_imu(&a,g,ac,1,1)); assert(a.gyro[0]==-.1 && a.accel[1]==-2 && a.accel[2]==9.81);
    assert(!bipe_app_imu(&a,NULL,ac,2,2)); assert(a.supervisor.fault==BIPE_IMU);
    /* Checked wheel modulo convention and uint32 time wrap. */
    p=synthetic(); p.wheel_modulo_turn=true; bipe_app_init(&a,&p,hal);
    bipe_motor_sample m={.present=true,.valid=true,.time_ms=0xfffffffeu,.sequence=1,.position=6.27,.id=5,.temperature=25};
    assert(bipe_app_motor(&a,&m)); m.time_ms=3; m.sequence=2; m.position=.01;
    assert(bipe_app_motor(&a,&m)); assert(a.inputs.motor[4].position>6.28);
    /* M12 following: only on an explicit request while ACTIVE; a manual command or a
       fault cancels it; a cancelled follow does not resume the old manual command. */
    p=synthetic(); bipe_app_init(&a,&p,hal);
    assert(a.follower.mode==BIPE_FOLLOW_OFF && !bipe_app_follow_request(&a,true,0));
    active(&a,&h);
    assert(bipe_app_command(&a,.2,.1,a.qref,a.qdref,2111) && a.velocity==.2);
    assert(bipe_app_follow_request(&a,true,2111));
    assert(a.follower.mode==BIPE_FOLLOW_WAITING && a.velocity==0 && a.yaw_rate==0);
    uint32_t sequence=0;
    for(uint32_t t=2112;t<=2600;t++) {
        feed(&a,t);
        if(t%33u==0u) {
            bipe_target_report r={.sequence=++sequence,.tag_id=0,.detected=true,.range_m=1.0,.bearing_rad=0,.age_s=.13};
            enum bipe_follow_report used=bipe_app_target(&a,&r,t);
            assert(used==BIPE_FOLLOW_ACCEPTED || (t<2300u && used==BIPE_FOLLOW_TOO_OLD));
        }
        bipe_app_tick(&a,t);
    }
    assert(a.supervisor.state==BIPE_ACTIVE && a.follower.mode==BIPE_FOLLOW_FOLLOWING);
    assert(fabs(a.follow_velocity)<.05 && fabs(a.follow_yaw_rate)<.05);
    /* A repeated report is not host keep-alive. */
    uint32_t host=a.inputs.host_time_ms;
    bipe_target_report again={.sequence=sequence,.tag_id=0,.detected=true,.range_m=1.0,.bearing_rad=0,.age_s=.13};
    assert(bipe_app_target(&a,&again,2600)==BIPE_FOLLOW_REPEATED && a.inputs.host_time_ms==host);
    assert(bipe_app_command(&a,0,0,a.qref,a.qdref,2601) && a.follower.mode==BIPE_FOLLOW_OFF);
    /* An invalid follower config refuses the request. */
    bipe_follow_config kept=a.follow_config; a.follow_config.history_s=0;
    assert(!bipe_app_follow_request(&a,true,2602) && a.follower.mode==BIPE_FOLLOW_OFF);
    a.follow_config=kept;
    assert(bipe_app_follow_request(&a,true,2602) && a.follower.mode==BIPE_FOLLOW_WAITING);
    assert(!bipe_app_command(&a,NAN,0,a.qref,a.qdref,2603));
    assert(a.supervisor.state==BIPE_FAULT && a.follower.mode==BIPE_FOLLOW_OFF);
    assert(!bipe_app_follow_request(&a,true,2604));
    puts("application: 200Hz batches, startup/calibration, finite effort, 8 latched faults, TX rejection, deadline, invalid host, duplicate/discontinuous feedback, restart, real-profile gate, axes, wheel/time wrap, follow request and cancel, repeated report not keep-alive, invalid follow config pass");
#else
    bipe_app_init(&a,&p,hal); bipe_start(&a.supervisor); feed(&a,0); bipe_app_tick(&a,0);
    assert(!a.profile_valid && h.frames==0 && !h.permission && a.supervisor.fault==BIPE_PROFILE);
    p.synthetic=false; bipe_app_init(&a,&p,hal); bipe_start(&a.supervisor); feed(&a,0); bipe_app_tick(&a,0);
    assert(!a.profile_valid && h.frames==0 && !h.permission);
    puts("production application: complete synthetic and real numeric profiles cannot transmit or request power while hardware gate is false");
#endif
    return 0;
}
