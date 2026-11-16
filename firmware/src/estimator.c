#include "bipe.h"
#include "generated_config.h"
#include <math.h>
#include <string.h>

static bool all_finite(const double *v,unsigned n) {
    for(unsigned i=0;i<n;i++) if(!isfinite(v[i])) return false;
    return true;
}
static double norm(const double v[3]) { return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
bipe_sensor_config bipe_default_sensor_config(void) {
    bipe_sensor_config c={BIPE_CONTROL_DT,BIPE_VELOCITY_FILTER_TAU_S,
        BIPE_JOINT_VELOCITY_FILTER_TAU_S,BIPE_POSTURE_FILTER_TAU_S,
        BIPE_ATTITUDE_CORRECTION_TAU_S,BIPE_ACCEL_NORM_GATE_M_S2,BIPE_PREDICTION_DELAY,
        BIPE_LOCAL_JOINT_VELOCITY_FILTER_TAU_S,BIPE_GRAVITY,BIPE_WHEEL_RADIUS,
        BIPE_IMU_VELOCITY_FUSION_TAU_S,BIPE_IMU_VELOCITY_INNOVATION_LIMIT_M_S,
        BIPE_IMU_ACCEL_BIAS_TAU_S}; return c;
}
#if BIPE_SERIAL
/* Forward horizontal offset of the IMU from the mean axle at the estimated pitch. */
static bool imu_offset(const double q[4],double pitch,double *offset) {
    double com[2],axle[2],eq,scale;
    if(!bipe_kinematics(q,com,axle,&eq,&scale)) return false;
    double dx=BIPE_IMU_X-axle[0],dz=BIPE_IMU_Z-axle[1];
    *offset=cos(pitch)*dx+sin(pitch)*dz; return true;
}
#endif
bool bipe_estimator_init(bipe_estimator *s,const double a[3],const double encoder[6]) {
    if(!s || !a || !encoder || !all_finite(a,3) || !all_finite(encoder,6) || norm(a)<1) return false;
    memset(s,0,sizeof(*s)); s->pitch=atan2(-a[0],hypot(a[1],a[2])); s->roll=atan2(a[1],a[2]);
    memcpy(s->previous,encoder,sizeof(s->previous)); memcpy(s->posture,encoder,sizeof(s->posture));
#if BIPE_SERIAL
    s->fusion_ready=imu_offset(encoder,s->pitch,&s->imu_offset);
#endif
    s->initialized=true; return true;
}
bool bipe_estimator_step(bipe_estimator *s,const bipe_sensor_config *c,
    const double gyro[3],const double accel[3],const double encoder[6],bipe_estimate *out) {
    if(!s || !c || !gyro || !accel || !encoder || !out || !s->initialized) return false;
    memset(out,0,sizeof(*out));
    const double settings[]={c->dt,c->velocity_tau,c->joint_tau,c->posture_tau,c->correction_tau,
        c->accel_gate,c->prediction_delay,c->local_tau,c->gravity,c->wheel_radius,
        c->fusion_tau,c->innovation_limit,c->bias_tau};
    if(!all_finite(settings,13) || c->fusion_tau<0 ||
       (c->fusion_tau>0 && (!BIPE_SERIAL || !s->fusion_ready || c->innovation_limit<=0 ||
                            c->bias_tau<=0)) || c->dt<=0 || c->dt>0.02 || c->velocity_tau<=0 || c->joint_tau<=0 ||
       c->posture_tau<0 || c->correction_tau<=0 || c->accel_gate<0 || c->prediction_delay<0 ||
       c->local_tau<=0 || c->gravity<=0 || c->wheel_radius<=0 || !all_finite(gyro,3) ||
       !all_finite(accel,3) || !all_finite(encoder,6) || fabs(cos(s->pitch))<0.1) return false;
    for(unsigned j=0;j<6;j++)
        if(fabs(encoder[j]-s->previous[j])>BIPE_MAX_ENCODER_SPEED*c->dt+0.0035) return false;
    double sr=sin(s->roll),cr=cos(s->roll),dt=c->dt;
    double rr=gyro[0]+tan(s->pitch)*(gyro[1]*sr+gyro[2]*cr);
    double pr=gyro[1]*cr-gyro[2]*sr,yr=(gyro[1]*sr+gyro[2]*cr)/cos(s->pitch);
    s->pitch+=pr*dt; s->roll+=rr*dt; s->yaw+=yr*dt;
    double f[3]={accel[0],accel[1],accel[2]};
#if BIPE_SERIAL
    if(c->fusion_tau>0) {
        /* Gravity alone: remove the forward horizontal acceleration, whose body-frame
           direction is (cos p, sin p sin r, sin p cos r), so a sustained acceleration does not
           pull the attitude towards atan(a/g). Only a drive's acceleration: in a push the
           low-passed wheel acceleration is not the IMU's. */
        double limit=fmax(s->compensation_limit,0);
        double a=fmax(-limit,fmin(limit,s->imu_acceleration));
        double sp=sin(s->pitch),cp=cos(s->pitch);
        f[0]-=a*cp; f[1]-=a*sp*sr; f[2]-=a*sp*cr;
    }
#endif
    if(fabs(norm(f)-c->gravity)<c->accel_gate) {
        double alpha=-expm1(-dt/c->correction_tau);
        s->pitch+=alpha*(atan2(-f[0],hypot(f[1],f[2]))-s->pitch);
        s->roll+=alpha*(atan2(f[1],f[2])-s->roll);
    }
    double raw[6],alpha=-expm1(-dt/c->velocity_tau),ja=-expm1(-dt/c->joint_tau);
    for(unsigned j=0;j<6;j++) {
        raw[j]=(encoder[j]-s->previous[j])/dt; s->previous[j]=encoder[j];
        s->rates[j]+=(j<4?ja:alpha)*(raw[j]-s->rates[j]);
        if(j<4) s->posture[j]+= (c->posture_tau>0?-expm1(-dt/c->posture_tau):1)*(encoder[j]-s->posture[j]);
    }
#if BIPE_FIVEBAR || BIPE_SERIAL
    double carrier[2];
    if(!bipe_carrier_rates(encoder,raw,carrier)) return false;
    double rolling=(raw[4]+raw[5]+2*gyro[1]-carrier[0]-carrier[1])/2;
#else
    double rolling=(raw[4]+raw[5]+2*gyro[1]+raw[0]+raw[1]+raw[2]+raw[3])/2;
#endif
    double raw_velocity=c->wheel_radius*rolling;
#if BIPE_SERIAL
    if(c->fusion_tau>0) {
        /* Predict with the forward horizontal specific force, correct towards wheel odometry
           moved to the IMU point; a second state estimates the forward accelerometer bias. */
        double offset;
        if(!imu_offset(encoder,s->pitch,&offset)) return false;
        double offset_rate=(offset-s->imu_offset)/dt; s->imu_offset=offset;
        double vertical=accel[1]*sin(s->roll)+accel[2]*cos(s->roll);
        s->imu_velocity+=dt*(accel[0]*cos(s->pitch)+vertical*sin(s->pitch)-s->accel_bias);
        double odometry=raw_velocity+offset_rate;
        s->imu_acceleration+=-expm1(-dt/c->fusion_tau)*((odometry-s->odometry)/dt-s->imu_acceleration);
        s->odometry=odometry;
        double innovation=fmin(fmax(odometry-s->imu_velocity,
            -c->innovation_limit),c->innovation_limit);
        s->imu_velocity+=-expm1(-dt/c->fusion_tau)*innovation;
        s->accel_bias-=dt*innovation/(c->fusion_tau*c->bias_tau);
        raw_velocity=s->imu_velocity-offset_rate;
    }
#endif
    s->velocity+=alpha*(raw_velocity-s->velocity);
    s->x+=s->velocity*dt*cos(s->yaw); s->y+=s->velocity*dt*sin(s->yaw);
    out->pitch=s->pitch+pr*c->prediction_delay; out->roll=s->roll+rr*c->prediction_delay;
    out->yaw=s->yaw+yr*c->prediction_delay; out->pitch_rate=pr; out->yaw_rate=yr;
    out->x=s->x; out->y=s->y; out->velocity=s->velocity;
    out->accel_norm=norm(accel);
    memcpy(out->q,s->posture,sizeof(out->q)); memcpy(out->qd,s->rates,sizeof(out->qd));
    memcpy(out->wheel_speed,s->rates+4,sizeof(out->wheel_speed)); return true;
}
bool bipe_local_step(bipe_local_feedback *s,const double q[4],double dt,double tau,double rates[4]) {
    if(!s || !q || !rates || !all_finite(q,4) || !isfinite(dt) || dt<=0 || dt>0.02 ||
       !isfinite(tau) || tau<=0) return false;
    if(!s->initialized) { memset(s,0,sizeof(*s)); memcpy(s->previous,q,sizeof(s->previous)); s->initialized=true; }
    for(unsigned j=0;j<4;j++)
        if(fabs(q[j]-s->previous[j])>BIPE_MAX_ENCODER_SPEED*dt+0.0002) return false;
    for(unsigned j=0;j<4;j++) {
        s->rates[j]+=-expm1(-dt/tau)*((q[j]-s->previous[j])/dt-s->rates[j]);
        s->previous[j]=q[j]; rates[j]=s->rates[j];
    } return true;
}
bool bipe_calibrate(bipe_calibration *s,const double gyro[3],const double accel[3],
    bool stationary,uint32_t now,uint32_t sequence,double bias[3],double gravity_sample[3]) {
    if(!s || !gyro || !accel || !bias || !gravity_sample) return false;
    if(!stationary || !all_finite(gyro,3) || !all_finite(accel,3) || norm(gyro)>BIPE_STATIONARY_GYRO ||
       fabs(norm(accel)-BIPE_GRAVITY)>BIPE_STATIONARY_ACCEL_TOLERANCE) {
        memset(s,0,sizeof(*s)); return false;
    }
    if(s->started && now-s->last_ms>(uint32_t)BIPE_IMU_MS) memset(s,0,sizeof(*s));
    if(s->started && sequence==s->last_sequence) return false;
    if(!s->started) { s->started=true; s->since_ms=now; }
    s->last_sequence=sequence; s->last_ms=now; s->samples++;
    for(unsigned j=0;j<3;j++) { s->gyro_sum[j]+=gyro[j]; s->accel_sum[j]+=accel[j]; }
    if(now-s->since_ms<(uint32_t)BIPE_CALIBRATION_MS || s->samples<200) return false;
    for(unsigned j=0;j<3;j++) { bias[j]=s->gyro_sum[j]/s->samples; gravity_sample[j]=s->accel_sum[j]/s->samples; }
    return true;
}
