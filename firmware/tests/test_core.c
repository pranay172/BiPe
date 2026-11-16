#include "bipe.h"
#include "generated_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static bipe_settings settings;
static bipe_inputs healthy(uint32_t now) {
    bipe_inputs in={0}; in.imu_present=true; in.imu_valid=true;
    in.imu_time_ms=now; in.imu_sequence=now; in.host_time_ms=now;
    in.estop_closed=true; in.power_good=true; in.bus_V=24; in.acceleration_norm=9.81;
    for(unsigned i=0;i<6;i++) {
        in.motor[i].id=(uint8_t)(i+1); in.motor[i].present=true;
        in.motor[i].valid=true; in.motor[i].time_ms=now; in.motor[i].temperature=25;
    }
    return in;
}
static void active(bipe_supervisor *s) {
    bipe_init(s); bipe_start(s);
    for(uint32_t t=0;t<=2001;t++) {
        bipe_inputs in=healthy(t); bipe_tick(s,&settings,&in,t,true);
    }
    assert(s->state==BIPE_READY && !s->torque_enabled);
    bipe_arm(s); bipe_inputs in=healthy(2002); bipe_tick(s,&settings,&in,2002,true);
    assert(s->state==BIPE_ACTIVE && s->torque_enabled);
}
int main(void) {
    settings=bipe_default_settings();
    /* Deliberately synthetic ranges: these are not AK40 variant settings. */
    bipe_ranges r={{0,4095},{0,4095},{0,65535},{0,4095},{0,4095}};
    bipe_mit c={0x123,0x456,0x789a,0xbcd,0xef0}; bipe_frame f;
    uint8_t golden[]={0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0};
    assert(bipe_pack(6,&r,&c,&f)); assert(f.id==0x806 && f.extended && f.dlc==8);
    assert(!memcmp(f.data,golden,8));
    c.kp=NAN; assert(!bipe_pack(6,&r,&c,&f)); c.kp=4096; assert(!bipe_pack(6,&r,&c,&f));
    bipe_ranges bad=r; bad.kp=(bipe_range){-1e308,1e308};
    c.kp=0; assert(!bipe_pack(6,&bad,&c,&f));
    c=(bipe_mit){4095,4095,65535,4095,4095}; assert(bipe_pack(6,&r,&c,&f));
    for(unsigned j=0;j<8;j++) assert(f.data[j]==255);
    f=(bipe_frame){.id=0x2906,.dlc=8,.extended=true,
                  .data={0x03,0x84,0x00,0x0e,0xff,0x9c,0xec,0}};
    bipe_feedback fb;
    assert(bipe_decode(&f,14,10,&fb)); assert(fabs(fb.position_rad-1.57079632679)<1e-9);
    assert(fabs(fb.velocity_rad_s-0.10471975512)<1e-9 && fb.iq_A==-1 && fb.temperature_C==-20);
    f.extended=false; assert(!bipe_decode(&f,14,10,&fb)); f.extended=true;
    f.remote=true; assert(!bipe_decode(&f,14,10,&fb)); f.remote=false;
    f.dlc=7; assert(!bipe_decode(&f,14,10,&fb));
    f=bipe_disable(6); assert(f.id==0xf06 && f.dlc==0 && f.extended);
    double total; assert(bipe_unwrap(6.27,0.01,6.27,0.005,45.553,&total));
    assert(total>6.28); assert(!bipe_unwrap(0,1,0,0.005,45.553,&total));
    assert(!bipe_unwrap(0,0.1,0,0.1,45.553,&total));
    bipe_supervisor s; bipe_init(&s); bipe_arm(&s); bipe_inputs in=healthy(0);
    bipe_tick(&s,&settings,&in,0,true); assert(s.state==BIPE_OFF && !s.torque_enabled);
    bipe_start(&s); bipe_tick(&s,&settings,&in,0,BIPE_HARDWARE_QUALIFIED);
    assert(s.state==BIPE_FAULT && s.fault==BIPE_PROFILE && !s.power_request);
    for(unsigned failure=0;failure<11;failure++) {
        active(&s); in=healthy(2003); enum bipe_fault expected=BIPE_OK; uint32_t now=2003;
        switch(failure) {
        case 0: in.imu_time_ms=1992; expected=BIPE_IMU; break;
        case 1: in.motor[2].time_ms=1982; expected=BIPE_FEEDBACK; break;
        case 2: in.bus_off=true; expected=BIPE_BUS_OFF; break;
        case 3: in.bus_V=17.9; expected=BIPE_VOLTAGE; break;
        case 4: in.motor[4].temperature=70.1; expected=BIPE_TEMPERATURE; break;
        case 5: in.estop_closed=false; expected=BIPE_ESTOP; break;
        case 6: now=2013; expected=BIPE_DEADLINE; break;
        case 7: in.attitude_error=0.53; expected=BIPE_ATTITUDE; break;
        case 8: in.motor[0].position=NAN; expected=BIPE_ENCODER; break;
        case 9: in.motor[1].id=1; expected=BIPE_FEEDBACK; break;
        case 10: in.motor[5].error=5; expected=BIPE_DRIVER; break;
        }
        bipe_tick(&s,&settings,&in,now,true);
        assert(s.state==BIPE_FAULT && s.fault==expected && !s.torque_enabled && !s.power_request);
        in=healthy(now+1); bipe_tick(&s,&settings,&in,now+1,true);
        assert(s.state==BIPE_FAULT); bipe_reset(&s);
        bipe_tick(&s,&settings,&in,now+1,true); assert(s.state==BIPE_OFF);
    }
    active(&s);
    for(uint32_t now=2003;now<=2800;now++) {
        in=healthy(now); in.host_time_ms=2002; bipe_tick(&s,&settings,&in,now,true);
        if(now==2253) assert(s.state==BIPE_STOPPING && s.torque_enabled);
    }
    assert(s.state==BIPE_READY && !s.torque_enabled);
    puts("core: golden codec, endpoint/rejection, rollover, startup, 11 latched faults, host stop pass");
    return 0;
}
