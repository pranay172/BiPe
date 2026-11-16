/* Transport/application boundary. Accepted sensing/control core is unchanged. */
#include "application.h"
#include "generated_config.h"
#include <math.h>
#include <string.h>
#ifndef BIPE_ALLOW_SYNTHETIC_APPLICATION
#define BIPE_ALLOW_SYNTHETIC_APPLICATION 0
#endif

static bool finite_array(const double *v, unsigned n) {
    if(!v) return false;
    for(unsigned i=0;i<n;i++) if(!isfinite(v[i])) return false;
    return true;
}
static void fail(bipe_application *a, enum bipe_fault reason) {
    a->supervisor.state=BIPE_FAULT; a->supervisor.fault=reason;
    a->supervisor.power_request=false; a->supervisor.torque_enabled=false;
    a->heartbeat=false;
    if(a->hal.power) a->hal.power(a->hal.context,false,false);
    if(a->commands_started && a->hal.disable_and_cancel_pending)
        (void)a->hal.disable_and_cancel_pending(a->hal.context);
    a->commands_started=false;
    bipe_follow_request(&a->follower,false);
    a->follow_velocity=0; a->follow_yaw_rate=0;
}
static bool profile_valid(const bipe_app_profile *p) {
    if(!p || p->evidence!=BIPE_APP_REQUIRED_EVIDENCE ||
       (p->synthetic ? !BIPE_ALLOW_SYNTHETIC_APPLICATION : !BIPE_HARDWARE_QUALIFIED) ||
       !isfinite(p->output_Nm_per_effort) || p->output_Nm_per_effort<=0 ||
       !isfinite(p->minimum_bus_V) || !isfinite(p->maximum_bus_V) ||
       p->minimum_bus_V<=0 || p->maximum_bus_V<=p->minimum_bus_V ||
       !finite_array(p->encoder_sign,6) || !finite_array(p->encoder_zero,6) ||
       !finite_array(p->imu_to_body,9)) return false;
    for(unsigned j=0;j<6;j++) if(fabs(p->encoder_sign[j])!=1) return false;
    for(unsigned r=0;r<3;r++) for(unsigned c=0;c<3;c++) {
        double dot=0;
        for(unsigned j=0;j<3;j++) dot+=p->imu_to_body[3*r+j]*p->imu_to_body[3*c+j];
        if(fabs(dot-(r==c?1:0))>1e-8) return false;
    }
    const double *m=p->imu_to_body;
    double determinant=m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6]);
    if(fabs(determinant-1)>1e-8) return false;
    const bipe_range ranges[5]={p->ranges.kp,p->ranges.kd,p->ranges.position,p->ranges.velocity,p->ranges.effort};
    for(unsigned j=0;j<5;j++)
        if(!isfinite(ranges[j].lo) || !isfinite(ranges[j].hi) ||
           !isfinite(ranges[j].hi-ranges[j].lo) || ranges[j].hi<=ranges[j].lo) return false;
    double effort=BIPE_JOINT_PEAK/p->output_Nm_per_effort;
    return p->ranges.kp.lo==0 && p->ranges.kd.lo==0 && p->ranges.position.lo<=0 && p->ranges.position.hi>=0 &&
        p->ranges.velocity.lo<=0 && p->ranges.velocity.hi>=0 &&
        p->ranges.kp.hi>=BIPE_JOINT_KP/p->output_Nm_per_effort &&
        p->ranges.kd.hi>=BIPE_JOINT_KD/p->output_Nm_per_effort &&
        p->ranges.effort.lo<=-effort && p->ranges.effort.hi>=effort;
}
bool bipe_app_profile_authorized(const bipe_application *a) {
    return profile_valid(&a->profile) && a->hal.send && a->hal.power && a->hal.disable_and_cancel_pending;
}
void bipe_app_init(bipe_application *a,const bipe_app_profile *profile,bipe_app_hal hal) {
    memset(a,0,sizeof(*a)); a->hal=hal;
    if(profile) a->profile=*profile;
    a->profile_valid=profile_valid(profile) && hal.send && hal.power && hal.disable_and_cancel_pending;
    const double q[4]={0.6981317007977318,-1.3962634015954636,0.6981317007977318,-1.3962634015954636};
    memcpy(a->qref,q,sizeof(q));
    a->follow_config=bipe_follow_default_config();
    bipe_follow_init(&a->follower);
    if(hal.power) hal.power(hal.context,false,false);
}
void bipe_app_reset(bipe_application *a) {
    bipe_app_profile profile=a->profile; bipe_app_hal hal=a->hal;
    if(hal.power) hal.power(hal.context,false,false);
    if(a->commands_started && hal.disable_and_cancel_pending)
        (void)hal.disable_and_cancel_pending(hal.context);
    bipe_app_init(a,&profile,hal);
}
bool bipe_app_motor(bipe_application *a,const bipe_motor_sample *sample) {
    if(!sample || sample->id<1 || sample->id>6 || !sample->valid ||
       !sample->present || !isfinite(sample->position) || !isfinite(sample->speed) ||
       !isfinite(sample->temperature)) { fail(a,BIPE_FEEDBACK); return false; }
    unsigned j=(unsigned)sample->id-1;
    const bipe_motor_sample *last=&a->inputs.motor[j];
    if(last->present && (sample->sequence==last->sequence || sample->time_ms==last->time_ms)) return false;
    double total=sample->position;
    if(last->present) {
        double dt=(sample->time_ms-last->time_ms)*.001;
        if(dt<=0 || dt>.02) { fail(a,BIPE_FEEDBACK); return false; }
        if(a->profile.wheel_modulo_turn && j>=4) {
            if(!bipe_unwrap(a->wrapped[j],sample->position,a->unwrapped[j],dt,BIPE_MAX_ENCODER_SPEED,&total)) {
                fail(a,BIPE_ENCODER); return false;
            }
        } else if(fabs(total-a->unwrapped[j])>BIPE_MAX_ENCODER_SPEED*dt+.0035) {
            fail(a,BIPE_ENCODER); return false;
        }
    }
    a->wrapped[j]=sample->position; a->unwrapped[j]=total;
    a->inputs.motor[j]=*sample;
    a->inputs.motor[j].position=(total-a->profile.encoder_zero[j])*a->profile.encoder_sign[j];
    a->inputs.motor[j].speed=sample->speed*a->profile.encoder_sign[j];
    return true;
}
bool bipe_app_imu(bipe_application *a,const double gyro[3],const double accel[3],uint32_t now,uint32_t sequence) {
    if(!finite_array(gyro,3) || !finite_array(accel,3)) { fail(a,BIPE_IMU); return false; }
    if(a->inputs.imu_present && (sequence==a->inputs.imu_sequence || now==a->inputs.imu_time_ms)) return false;
    double g2=0,a2=0;
    for(unsigned r=0;r<3;r++) {
        a->gyro[r]=0; a->accel[r]=0;
        for(unsigned c=0;c<3;c++) {
            a->gyro[r]+=a->profile.imu_to_body[3*r+c]*gyro[c];
            a->accel[r]+=a->profile.imu_to_body[3*r+c]*accel[c];
        }
        g2+=a->gyro[r]*a->gyro[r]; a2+=a->accel[r]*a->accel[r];
    }
    a->inputs.gyro_norm=sqrt(g2); a->inputs.acceleration_norm=sqrt(a2);
    a->inputs.imu_present=true; a->inputs.imu_valid=true;
    a->inputs.imu_time_ms=now; a->inputs.imu_sequence=sequence;
    return true;
}
bool bipe_app_command(bipe_application *a,double velocity,double yaw,const double q[4],const double qd[4],uint32_t now) {
    if(!isfinite(velocity) || fabs(velocity)>BIPE_DRIVE_SPEED || !isfinite(yaw) || fabs(yaw)>1.0 ||
       !finite_array(q,4) || !finite_array(qd,4) || !bipe_joint_limits(q,0)) {
        fail(a,BIPE_HOST); return false;
    }
    for(unsigned j=0;j<4;j++) if(fabs(qd[j])>BIPE_MAX_ENCODER_SPEED) { fail(a,BIPE_HOST); return false; }
    bipe_follow_request(&a->follower,false); /* a manual command cancels following */
    a->follow_velocity=0; a->follow_yaw_rate=0;
    a->velocity=velocity; a->yaw_rate=yaw; memcpy(a->qref,q,sizeof(a->qref)); memcpy(a->qdref,qd,sizeof(a->qdref));
    a->inputs.host_time_ms=now; return true;
}
bool bipe_app_follow_request(bipe_application *a,bool on,uint32_t now) {
    if(a->supervisor.state==BIPE_FAULT) return false;
    if(on && (a->supervisor.state!=BIPE_ACTIVE || !bipe_follow_config_valid(&a->follow_config))) return false;
    bipe_follow_request(&a->follower,on);
    a->velocity=0; a->yaw_rate=0; a->follow_velocity=0; a->follow_yaw_rate=0;
    a->inputs.host_time_ms=now; return true;
}
enum bipe_follow_report bipe_app_target(bipe_application *a,const bipe_target_report *r,uint32_t now) {
    if(!r) return BIPE_FOLLOW_INVALID;
    enum bipe_follow_report used=bipe_follow_report_in(&a->follower,&a->follow_config,r,now*.001);
    /* A repeated or invalid report is not a sign of a live host. */
    if(a->follower.requested && used!=BIPE_FOLLOW_INVALID && used!=BIPE_FOLLOW_REPEATED) a->inputs.host_time_ms=now;
    return used;
}
void bipe_app_tick(bipe_application *a,uint32_t now) {
    if(a->supervisor.state==BIPE_FAULT) { if(a->hal.power) a->hal.power(a->hal.context,false,false); return; }
    /* Validation is rechecked each tick; a caller cannot turn a malformed
       runtime profile into an authorization by changing one cached boolean. */
    a->profile_valid=profile_valid(&a->profile) && a->hal.send && a->hal.power && a->hal.disable_and_cancel_pending;
    bipe_settings settings=bipe_default_settings();
    if(a->profile_valid) {
        if(settings.bus_arm_min<a->profile.minimum_bus_V) settings.bus_arm_min=a->profile.minimum_bus_V;
        if(settings.bus_arm_max>a->profile.maximum_bus_V) settings.bus_arm_max=a->profile.maximum_bus_V;
        if(settings.bus_fault_min<a->profile.minimum_bus_V) settings.bus_fault_min=a->profile.minimum_bus_V;
        if(settings.bus_fault_max>a->profile.maximum_bus_V) settings.bus_fault_max=a->profile.maximum_bus_V;
    }
    if(a->supervisor.arm_requested && (!a->estimator_ready || a->driver_limits_ack_mask!=0x3fu)) {
        fail(a,BIPE_PROFILE); return;
    }
    bipe_tick(&a->supervisor,&settings,&a->inputs,now,a->profile_valid);
    if(a->supervisor.state==BIPE_FAULT) { fail(a,a->supervisor.fault); return; }
    if(!a->supervisor.power_request) { if(a->hal.power) a->hal.power(a->hal.context,false,false); return; }
    if(a->inputs.bus_off) { fail(a,BIPE_BUS_OFF); return; }
    if(!a->supervisor.torque_enabled && a->commands_started) {
        if(!a->hal.disable_and_cancel_pending(a->hal.context)) { fail(a,BIPE_BUS_OFF); return; }
        a->commands_started=false;
    }
    if(!a->control_seen || now-a->last_control_ms>=5u) {
        if(a->control_seen && now-a->last_control_ms>5u) { fail(a,BIPE_DEADLINE); return; }
        double encoder[6]; bool stationary=true;
        for(unsigned j=0;j<6;j++) {
            encoder[j]=a->inputs.motor[j].position;
            stationary=stationary && a->inputs.motor[j].present && now-a->inputs.motor[j].time_ms<=settings.feedback_ms &&
                fabs(a->inputs.motor[j].speed)<.05;
        }
        if(!a->estimator_ready) {
            double gravity[3];
            if(bipe_calibrate(&a->calibration,a->gyro,a->accel,stationary,now,a->inputs.imu_sequence,a->bias,gravity))
                a->estimator_ready=bipe_estimator_init(&a->estimator,gravity,encoder);
        } else {
            double corrected[3]; for(unsigned j=0;j<3;j++) corrected[j]=a->gyro[j]-a->bias[j];
            bipe_sensor_config cfg=bipe_default_sensor_config();
            cfg.dt=a->control_seen?(now-a->last_control_ms)*.001:.005;
            if(!bipe_estimator_step(&a->estimator,&cfg,corrected,a->accel,encoder,&a->estimate) || !bipe_joint_limits(a->estimate.q,.01)) {
                fail(a,BIPE_ENCODER); return;
            }
            double com[2], axle[2], equilibrium, scale;
            if(!bipe_kinematics(a->estimate.q,com,axle,&equilibrium,&scale)) { fail(a,BIPE_ENCODER); return; }
            a->inputs.attitude_error=a->estimate.pitch-equilibrium;
            if(fabs(a->inputs.attitude_error)>settings.attitude_max) { fail(a,BIPE_ATTITUDE); return; }
            bool allowed=a->supervisor.state==BIPE_ACTIVE && a->supervisor.torque_enabled;
            if(!bipe_follow_step(&a->follower,&a->follow_config,now*.001,a->estimate.x,a->estimate.y,
                a->estimate.yaw,a->inputs.attitude_error,a->estimate.velocity,allowed,
                &a->follow_velocity,&a->follow_yaw_rate)) {
                bipe_follow_request(&a->follower,false); /* a failed step cancels following */
                a->follow_velocity=0; a->follow_yaw_rate=0;
            }
        }
        a->control_seen=true; a->last_control_ms=now;
        a->estimator.compensation_limit=0; /* set again below while the balance drives */
        if(a->supervisor.torque_enabled && a->estimator_ready) {
            if(a->driver_limits_ack_mask!=0x3fu) { fail(a,BIPE_PROFILE); return; }
            double torque[6],ff[4],reference;
            bool follow=a->follower.mode!=BIPE_FOLLOW_OFF;
            double v=a->supervisor.state==BIPE_STOPPING?0:follow?a->follow_velocity:a->velocity;
            double yaw=a->supervisor.state==BIPE_STOPPING?0:follow?a->follow_yaw_rate:a->yaw_rate;
            if(!bipe_control_step(&a->balance,&a->estimate,v,yaw,a->qref,a->qdref,.005,torque,ff,&reference)) { fail(a,BIPE_ENCODER); return; }
            a->estimator.compensation_limit=
                a->balance.previous_velocity!=0 || a->balance.braking ? BIPE_MAX_COMMAND_ACCELERATION : 0;
            bipe_frame frames[6];
            for(unsigned j=0;j<6;j++) {
                double sign=a->profile.encoder_sign[j],factor=a->profile.output_Nm_per_effort;
                bipe_mit command={0};
                if(j<4) {
                    command.kp=BIPE_JOINT_KP/factor; command.kd=BIPE_JOINT_KD/factor;
                    command.position=sign*a->qref[j]+a->profile.encoder_zero[j];
                    command.velocity=sign*a->qdref[j]; command.effort=sign*ff[j]/factor;
                } else command.effort=sign*torque[j]/factor;
                /* Build the complete batch before transmitting any part. The
                   qualified driver must enforce configured finite effort locally. */
                if(!bipe_pack((uint8_t)(j+1),&a->profile.ranges,&command,&frames[j])) { fail(a,BIPE_PROFILE); return; }
                a->last_effort[j]=torque[j];
            }
            a->commands_started=true;
            for(unsigned j=0;j<6;j++) if(!a->hal.send(a->hal.context,&frames[j])) { fail(a,BIPE_BUS_OFF); return; }
            a->sent_batches++;
        }
    }
    if(now-a->last_heartbeat_ms>=10u) { a->heartbeat=!a->heartbeat; a->last_heartbeat_ms=now; }
    a->hal.power(a->hal.context,true,a->heartbeat);
}
