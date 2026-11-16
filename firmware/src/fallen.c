#include "fallen.h"
#include <math.h>
#include <string.h>

static bool finite_n(const double *v, unsigned n) {
    for(unsigned i=0;i<n;i++) if(!isfinite(v[i])) return false;
    return true;
}

bool bipe_fallen_step(bipe_fallen_estimator *s, const bipe_sensor_config *c,
    const double gyro[3], const double accel[3], const double encoder[6],
    bipe_estimate *out, bool *valid) {
    if(!s || !c || !gyro || !accel || !encoder || !out || !valid) return false;
    if(!finite_n(gyro,3) || !finite_n(accel,3) || !finite_n(encoder,6) ||
       !isfinite(c->dt) || c->dt<=0 || !isfinite(c->joint_tau) || c->joint_tau<=0) return false;
    if(!s->started) { memcpy(s->previous,encoder,sizeof(s->previous)); memset(s->rates,0,sizeof(s->rates)); s->started=true; }
    double a=c->dt/(c->joint_tau+c->dt);
    for(unsigned j=0;j<6;j++) {
        s->rates[j]+=a*((encoder[j]-s->previous[j])/c->dt-s->rates[j]);
        s->previous[j]=encoder[j];
    }
    /* M17: the estimator takes over only after BIPE_FALLEN_TAKEOVER_TICKS quiet ticks in a
       row (accelerometer within the estimator's gate of 1 g, every gyro axis under
       BIPE_FALLEN_TAKEOVER_RATE), so its attitude starts from gravity alone. Taking over on
       the first accepted tick could start it mid-flip from a reading of up to 1.6 g, and it
       then needed about 5 s to agree with the true pitch, past the get-up's handover
       deadline; a flip also passes through single quiet instants. Once it has taken over
       it runs as in balance. */
    double g_norm=sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    bool quiet=fabs(g_norm-c->gravity)<c->accel_gate && fabs(gyro[0])<BIPE_FALLEN_TAKEOVER_RATE &&
        fabs(gyro[1])<BIPE_FALLEN_TAKEOVER_RATE && fabs(gyro[2])<BIPE_FALLEN_TAKEOVER_RATE;
    if(!quiet) s->quiet=0; else if(s->quiet<UINT32_MAX) s->quiet++;
    *valid=s->estimator.initialized && (s->accepted>0 || s->quiet>=BIPE_FALLEN_TAKEOVER_TICKS) &&
        bipe_estimator_step(&s->estimator,c,gyro,accel,encoder,out);
    if(*valid) { s->accepted++; return true; }
    s->refused++; s->accepted=0;
    memset(out,0,sizeof(*out));
    out->pitch=atan2(-accel[0],hypot(accel[1],accel[2]));
    out->roll=atan2(accel[1],accel[2]);
    out->pitch_rate=gyro[1]; out->yaw_rate=gyro[2];
    for(unsigned j=0;j<4;j++) { out->q[j]=encoder[j]; out->qd[j]=s->rates[j]; }
    out->wheel_speed[0]=s->rates[4]; out->wheel_speed[1]=s->rates[5];
    out->accel_norm=sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    (void)bipe_estimator_init(&s->estimator,accel,encoder);
    return true;
}
