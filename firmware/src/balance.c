#include "bipe.h"
#include "generated_config.h"
#include <math.h>

/* M12 options; zero keeps the earlier rate-limited reference without feedforward. */
#ifndef BIPE_COMMAND_JERK_M_S3
#define BIPE_COMMAND_JERK_M_S3 0.0
#endif
#ifndef BIPE_ACCELERATION_FEEDFORWARD
#define BIPE_ACCELERATION_FEEDFORWARD 0.0
#endif
#ifndef BIPE_HOLD_COMMAND_ACCELERATION
#define BIPE_HOLD_COMMAND_ACCELERATION BIPE_MAX_COMMAND_ACCELERATION
#endif

static double clamp(double x,double lo,double hi) { return fmax(lo,fmin(hi,x)); }
/* Jerk-limited reference: the acceleration slews toward the largest value that still ramps it
   to zero at the target, so the reference does not overshoot and its lean is continuous. */
static void shape_reference(double *vref,double *acceleration,double target,double limit,
                            double jerk,double dt) {
    double error=target-*vref, step=jerk*dt;
    if(fabs(error)<=step*dt && fabs(*acceleration)<=step) { *vref=target; *acceleration=0; return; }
    double remaining=error-*acceleration*fabs(*acceleration)/(2*jerk);
    double want=copysign(sqrt(2*jerk*fabs(remaining)),remaining);
    *acceleration+=clamp(clamp(want,-limit,limit)-*acceleration,-step,step);
    *vref+=*acceleration*dt;
}
bool bipe_balance_step(bipe_balance *b,const bipe_observation *o,double velocity,
                       double yaw_rate,double dt,double wheels[2]) {
    if(!b || !o || !wheels || !isfinite(dt) || dt<=0 || dt>0.02 ||
       !isfinite(velocity) || !isfinite(yaw_rate) || fabs(velocity)>BIPE_DRIVE_SPEED ||
       fabs(yaw_rate)>1.0) return false;
    const double values[]={o->pitch,o->pitch_rate,o->yaw,o->yaw_rate,o->x,o->y,
        o->velocity,o->equilibrium,o->height_scale,o->wheel_speed[0],o->wheel_speed[1]};
    for(unsigned j=0;j<sizeof(values)/sizeof(values[0]);j++) if(!isfinite(values[j])) return false;
    /* With a jerk limit a stop command brakes the reference to rest, then holds there. */
    bool stopping=velocity==0 && b->previous_velocity!=0;
    bool settled=b->braking && b->vref==0 && b->command_acceleration==0;
    if(BIPE_COMMAND_JERK_M_S3>0) b->braking=(b->braking || stopping) && velocity==0 && !settled;
    bool moving=velocity!=0 || b->braking;
    if(!b->hold_initialized || (stopping && !b->braking) || settled) {
        b->hold_x=o->x; b->hold_y=o->y; b->hold_initialized=true;
    }
    b->previous_velocity=velocity; b->heading+=yaw_rate*dt;
    double target=velocity;
    if(!moving) target=clamp(BIPE_HOLD_KP*((b->hold_x-o->x)*cos(o->yaw)+
        (b->hold_y-o->y)*sin(o->yaw)),-BIPE_MAX_HOLD_SPEED,BIPE_MAX_HOLD_SPEED);
    if(BIPE_COMMAND_JERK_M_S3>0 && moving) {
        shape_reference(&b->vref,&b->command_acceleration,target,BIPE_MAX_COMMAND_ACCELERATION,
                        BIPE_COMMAND_JERK_M_S3,dt);
        b->acceleration_lean=BIPE_ACCELERATION_FEEDFORWARD*atan(b->command_acceleration/BIPE_GRAVITY);
    } else {
        /* The hold reference is the position loop's output: plain rate limit, no feedforward. */
        double step=(moving ? BIPE_MAX_COMMAND_ACCELERATION : BIPE_HOLD_COMMAND_ACCELERATION)*dt;
        b->command_acceleration=b->acceleration_lean=0;
        b->vref+=clamp(target-b->vref,-step,step);
    }
    double speed_kp=BIPE_SPEED_KP, speed_ki=BIPE_SPEED_KI;
#if BIPE_FIVEBAR || BIPE_SERIAL
    if(!moving) { speed_kp=BIPE_HOLD_SPEED_KP; speed_ki=BIPE_HOLD_SPEED_KI; }
#endif
    double error=b->vref-o->velocity;
    double pitch_kd=BIPE_PITCH_KD;
#if BIPE_FIVEBAR || BIPE_SERIAL
    if(moving) pitch_kd=BIPE_DRIVE_PITCH_KD;
#endif
    double candidate=clamp(b->integral+error*dt,-BIPE_SPEED_INTEGRAL_LIMIT,BIPE_SPEED_INTEGRAL_LIMIT);
    double lean=speed_kp*error+speed_ki*candidate;
#if BIPE_BALANCE_LQR
    (void)pitch_kd;
    for(unsigned j=0;j<4;j++) if(!isfinite(o->q[j]) || !isfinite(o->qd[j]) ||
        !isfinite(o->qref[j]) || !isfinite(o->qdref[j])) return false;
    double K[9]; unsigned lower=0;
    while(lower+1<BIPE_LQR_ROWS && o->height_scale>bipe_lqr_schedule[lower+1][0]) lower++;
    unsigned upper=lower+1<BIPE_LQR_ROWS?lower+1:lower;
    double span=bipe_lqr_schedule[upper][0]-bipe_lqr_schedule[lower][0];
    double weight=span>0?clamp((o->height_scale-bipe_lqr_schedule[lower][0])/span,0,1):0;
    for(unsigned j=0;j<9;j++) K[j]=(1-weight)*bipe_lqr_schedule[lower][j+1]+
        weight*bipe_lqr_schedule[upper][j+1];
    double x[9]={o->pitch-o->equilibrium,o->pitch_rate,o->velocity-b->vref,
        b->common_lag,candidate,
        (o->q[0]+o->q[2]-o->qref[0]-o->qref[2])/2,
        (o->q[1]+o->q[3]-o->qref[1]-o->qref[3])/2,
        (o->qd[0]+o->qd[2]-o->qdref[0]-o->qdref[2])/2,
        (o->qd[1]+o->qd[3]-o->qdref[1]-o->qdref[3])/2};
    double common=0; for(unsigned j=0;j<9;j++) common-=K[j]*x[j];
    if(fabs(common)<BIPE_WHEEL_PEAK || (-K[4]*error)*common<0) b->integral=candidate;
    common+=K[4]*(candidate-b->integral);
    (void)lean;
#else
#if BIPE_FIVEBAR || BIPE_SERIAL
    if(BIPE_SLOPE_OBSERVER_TAU_S>0) {
        /* dp/dt = sum(previous wheel torque)/r + F, F = (M (v + dd/dt) - p)/tau with d the CoM's
           horizontal offset from the axle, gives sin(slope) = -F/(M g); lean the CoM over the
           tilted contact, less the wheel-torque term's own lean, slew limited so that a push
           (an impulsive external force) cannot step it. */
        double com[2],axle[2],eq,scale;
        if(!bipe_kinematics(o->q,com,axle,&eq,&scale)) return false;
        double mass=BIPE_TOTAL_MASS,height=com[1]-axle[1];
        double offset=cos(o->pitch)*(com[0]-axle[0])+sin(o->pitch)*(com[1]-axle[1]);
        if(!b->slope_initialized) {
            b->slope_momentum=mass*o->velocity; b->slope_offset=offset; b->slope_initialized=true;
        }
        double offset_rate=(offset-b->slope_offset)/dt; b->slope_offset=offset;
        b->slope_momentum+=dt*(b->wheel_sum/BIPE_WHEEL_RADIUS+b->slope_force);
        b->slope_force=(mass*(o->velocity+offset_rate)-b->slope_momentum)/BIPE_SLOPE_OBSERVER_TAU_S;
        double limit=sin(BIPE_SLOPE_FEEDFORWARD_LIMIT_RAD);
        double sine=clamp(-b->slope_force/(mass*BIPE_GRAVITY),-limit,limit);
        double lean_target=asin(BIPE_WHEEL_RADIUS*sine/height)-
            mass*BIPE_GRAVITY*BIPE_WHEEL_RADIUS*sine/2/BIPE_PITCH_KP;
        double step=BIPE_SLOPE_LEAN_RATE_LIMIT_RAD_S*dt;
        b->slope_lean+=clamp(lean_target-b->slope_lean,-step,step);
    }
    double candidate_lean=clamp(lean,-BIPE_MAX_PITCH_OFFSET,BIPE_MAX_PITCH_OFFSET);
    double candidate_common=BIPE_PITCH_KP*(o->pitch-o->equilibrium-b->slope_lean-b->acceleration_lean-
        candidate_lean)+
        o->height_scale*pitch_kd*o->pitch_rate;
    bool wheel_unwinds=(-BIPE_PITCH_KP*speed_ki*error)*candidate_common<0;
    if((fabs(lean)<BIPE_MAX_PITCH_OFFSET || error*lean<0) &&
       (fabs(candidate_common)<BIPE_WHEEL_PEAK || wheel_unwinds)) b->integral=candidate;
#else
    if(fabs(lean)<BIPE_MAX_PITCH_OFFSET || error*lean<0) b->integral=candidate;
#endif
    lean=clamp(speed_kp*error+speed_ki*b->integral,-BIPE_MAX_PITCH_OFFSET,BIPE_MAX_PITCH_OFFSET);
#if BIPE_FIVEBAR || BIPE_SERIAL
    lean+=b->slope_lean;
#endif
    lean+=b->acceleration_lean;
    double common=BIPE_PITCH_KP*(o->pitch-o->equilibrium-lean)+
        o->height_scale*pitch_kd*o->pitch_rate;
#endif
    double heading_error=remainder(b->heading-o->yaw,6.28318530717958647692);
    double diff=BIPE_HEADING_KP*heading_error+BIPE_YAW_RATE_KD*(yaw_rate-o->yaw_rate);
    double available=fmax(0,BIPE_WHEEL_PEAK-fabs(common)); diff=clamp(diff,-available,available);
    b->wheel_sum=0;
    for(unsigned j=0;j<2;j++) {
        double torque=common+(j==0?-diff:diff), speed=o->wheel_speed[j];
        b->wheel_sum+=clamp(torque,-BIPE_WHEEL_PEAK,BIPE_WHEEL_PEAK);
        double blend=fmin(fabs(speed)/BIPE_FRICTION_VELOCITY_THRESHOLD_RAD_S,1);
        torque+=BIPE_WHEEL_FRICTION_COMPENSATION*((1-blend)*
            tanh(torque/BIPE_FRICTION_COMPENSATION_SCALE)+blend*
            tanh(speed/(BIPE_FRICTION_VELOCITY_THRESHOLD_RAD_S/4)));
        wheels[j]=clamp(torque,-BIPE_WHEEL_PEAK,BIPE_WHEEL_PEAK);
    }
    b->common_lag+=-expm1(-dt/BIPE_WHEEL_TAU)*((wheels[0]+wheels[1])/2-b->common_lag);
    return true;
}
