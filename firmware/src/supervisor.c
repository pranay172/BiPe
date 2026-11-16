#include "bipe.h"
#include "generated_config.h"
#include <math.h>
#include <string.h>

void bipe_init(bipe_supervisor *s) { memset(s,0,sizeof(*s)); }
bipe_settings bipe_default_settings(void) {
    return (bipe_settings){(unsigned)BIPE_FEEDBACK_MS,(unsigned)BIPE_IMU_MS,
        (unsigned)BIPE_HOST_MS,(unsigned)BIPE_STOP_MS,(unsigned)BIPE_DEADLINE_MS,
        (unsigned)BIPE_CALIBRATION_MS,BIPE_BUS_ARM_MIN,BIPE_BUS_ARM_MAX,
        BIPE_BUS_FAULT_MIN,BIPE_BUS_FAULT_MAX,BIPE_TEMPERATURE_MAX,
        BIPE_STATIONARY_GYRO,BIPE_STATIONARY_JOINT_SPEED,BIPE_ATTITUDE_MAX};
}
void bipe_start(bipe_supervisor *s) { if(s->state==BIPE_OFF) s->start_requested=true; }
void bipe_arm(bipe_supervisor *s) { if(s->state==BIPE_READY) s->arm_requested=true; }
void bipe_reset(bipe_supervisor *s) { bipe_init(s); }
bool bipe_begin_fall(bipe_supervisor *s,uint32_t now,unsigned maximum_ms) {
#if BIPE_FIVEBAR || BIPE_SERIAL
    if(!s || s->state!=BIPE_ACTIVE || !maximum_ms || maximum_ms>500) return false;
    s->state=BIPE_FALLING; s->stop_since=now; s->fall_stop_ms=maximum_ms;
    return true;
#else
    (void)s; (void)now; (void)maximum_ms; return false;
#endif
}
void bipe_end_fall(bipe_supervisor *s) {
    if(!s || (s->state!=BIPE_ACTIVE && s->state!=BIPE_FALLING)) return;
    s->state=BIPE_FALLEN; s->power_request=false; s->torque_enabled=false;
}
static void fault(bipe_supervisor *s, enum bipe_fault f) {
    s->state=BIPE_FAULT; s->fault=f; s->power_request=false; s->torque_enabled=false;
}
static enum bipe_fault health(const bipe_settings *c, const bipe_inputs *in,
                              uint32_t now,bool falling) {
    if (!in->estop_closed) return BIPE_ESTOP;
    if (in->bus_off) return BIPE_BUS_OFF;
    if (!in->power_good || !isfinite(in->bus_V) || in->bus_V<c->bus_fault_min ||
        in->bus_V>c->bus_fault_max) return BIPE_VOLTAGE;
    if (!in->imu_present || !in->imu_valid || now-in->imu_time_ms>c->imu_ms ||
        !isfinite(in->gyro_norm) || in->gyro_norm<0 || !isfinite(in->acceleration_norm) ||
        in->acceleration_norm<0 ||
        !isfinite(in->attitude_error)) return BIPE_IMU;
    if (!falling && fabs(in->attitude_error)>c->attitude_max) return BIPE_ATTITUDE;
    for (unsigned j=0;j<BIPE_MOTORS;j++) {
        const bipe_motor_sample *m=&in->motor[j];
        if (!m->present || !m->valid || m->id!=j+1 || now-m->time_ms>c->feedback_ms)
            return BIPE_FEEDBACK;
        if (!isfinite(m->position) || !isfinite(m->speed)) return BIPE_ENCODER;
        if (!isfinite(m->temperature) || m->temperature>c->temperature_max)
            return BIPE_TEMPERATURE;
        if (m->error) return BIPE_DRIVER;
    }
    return BIPE_OK;
}
void bipe_tick(bipe_supervisor *s, const bipe_settings *c, const bipe_inputs *in,
               uint32_t now, bool qualified) {
    if (s->state==BIPE_FAULT || s->state==BIPE_FALLEN) return;
    if (s->state==BIPE_OFF && !s->start_requested) {
        s->power_request=false; s->torque_enabled=false; return;
    }
    if (!qualified) { fault(s,BIPE_PROFILE); return; }
    if (s->tick_seen && now-s->last_tick>c->deadline_ms) { fault(s,BIPE_DEADLINE); return; }
    s->last_tick=now; s->tick_seen=true;
    if (s->state==BIPE_OFF) {
        s->start_requested=false; s->state=BIPE_DISCOVERING;
        s->calibration_since=now; s->power_request=true;
    }
    enum bipe_fault f=health(c,in,now,s->state==BIPE_FALLING);
    /* Give powered actuators a bounded discovery interval; never apply torque. */
    if(s->state==BIPE_DISCOVERING && f==BIPE_FEEDBACK &&
       now-s->calibration_since<(unsigned)BIPE_DISCOVERY_MS) return;
    if(f!=BIPE_OK) { fault(s,f); return; }
    if(s->state==BIPE_FALLING && now-s->stop_since>=s->fall_stop_ms) bipe_end_fall(s);
    if (s->state==BIPE_DISCOVERING) s->state=BIPE_CALIBRATING;
    if (s->state==BIPE_CALIBRATING) {
        bool stationary=in->gyro_norm<c->stationary_gyro &&
            fabs(in->acceleration_norm-BIPE_GRAVITY)<BIPE_STATIONARY_ACCEL_TOLERANCE &&
            in->bus_V>=c->bus_arm_min && in->bus_V<=c->bus_arm_max;
        for(unsigned j=0;j<4;j++)
            stationary=stationary && fabs(in->motor[j].speed)<c->stationary_joint_speed;
        for(unsigned j=4;j<6;j++) stationary=stationary && fabs(in->motor[j].speed)<BIPE_STATIONARY_WHEEL_SPEED;
        if (!stationary) {
            s->calibrating_stationary=false; s->calibration_samples=0;
        } else {
            if (!s->calibrating_stationary) {
                s->calibration_since=now; s->calibrating_stationary=true;
                s->last_imu_sequence=in->imu_sequence; s->calibration_samples=0;
            } else if (in->imu_sequence!=s->last_imu_sequence) {
                s->calibration_samples++; s->last_imu_sequence=in->imu_sequence;
            }
            if(now-s->calibration_since>=c->calibration_ms &&
               s->calibration_samples>=c->calibration_ms/2) s->state=BIPE_READY;
        }
    }
    if(s->state==BIPE_READY && s->arm_requested) {
        s->arm_requested=false;
        if(in->bus_V<c->bus_arm_min || in->bus_V>c->bus_arm_max) {
            fault(s,BIPE_VOLTAGE); return;
        }
        if(now-in->host_time_ms>c->host_ms) { fault(s,BIPE_HOST); return; }
        s->state=BIPE_ACTIVE;
    }
    if(s->state==BIPE_ACTIVE && now-in->host_time_ms>c->host_ms) {
        s->state=BIPE_STOPPING; s->stop_since=now;
    }
    if(s->state==BIPE_STOPPING && now-s->stop_since>=c->stop_ms) {
        s->state=BIPE_READY; s->arm_requested=false;
    }
    s->torque_enabled=s->state==BIPE_ACTIVE || s->state==BIPE_STOPPING || s->state==BIPE_FALLING;
}
