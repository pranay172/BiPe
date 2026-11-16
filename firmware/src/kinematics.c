#include "bipe.h"
#include "generated_config.h"
#include <math.h>
#include <string.h>

#if BIPE_SERIAL
/* The 13 CAD rigid bodies of the serial profile; indices within each side are
   thigh, knee drive, shin, wheel, thigh rotor, knee rotor. */
static const double masses[13]={BIPE_CHASSIS_MASS,
    BIPE_LEFT_THIGH_MASS,BIPE_LEFT_KNEEDRIVE_MASS,BIPE_LEFT_SHIN_MASS,BIPE_LEFT_WHEEL_MASS,
    BIPE_LEFT_THIGHROTOR_MASS,BIPE_LEFT_KNEEROTOR_MASS,
    BIPE_RIGHT_THIGH_MASS,BIPE_RIGHT_KNEEDRIVE_MASS,BIPE_RIGHT_SHIN_MASS,BIPE_RIGHT_WHEEL_MASS,
    BIPE_RIGHT_THIGHROTOR_MASS,BIPE_RIGHT_KNEEROTOR_MASS};
static const double centers[13][2]={{BIPE_CHASSIS_X,BIPE_CHASSIS_Z},
    {BIPE_LEFT_THIGH_X,BIPE_LEFT_THIGH_Z},{BIPE_LEFT_KNEEDRIVE_X,BIPE_LEFT_KNEEDRIVE_Z},
    {BIPE_LEFT_SHIN_X,BIPE_LEFT_SHIN_Z},{BIPE_LEFT_WHEEL_X,BIPE_LEFT_WHEEL_Z},
    {BIPE_LEFT_THIGHROTOR_X,BIPE_LEFT_THIGHROTOR_Z},{BIPE_LEFT_KNEEROTOR_X,BIPE_LEFT_KNEEROTOR_Z},
    {BIPE_RIGHT_THIGH_X,BIPE_RIGHT_THIGH_Z},{BIPE_RIGHT_KNEEDRIVE_X,BIPE_RIGHT_KNEEDRIVE_Z},
    {BIPE_RIGHT_SHIN_X,BIPE_RIGHT_SHIN_Z},{BIPE_RIGHT_WHEEL_X,BIPE_RIGHT_WHEEL_Z},
    {BIPE_RIGHT_THIGHROTOR_X,BIPE_RIGHT_THIGHROTOR_Z},{BIPE_RIGHT_KNEEROTOR_X,BIPE_RIGHT_KNEEROTOR_Z}};
#elif !BIPE_FIVEBAR
static const double masses[7]={BIPE_CHASSIS_MASS,BIPE_LEFT_UPPER_MASS,BIPE_LEFT_LOWER_MASS,
    BIPE_LEFT_WHEEL_MASS,BIPE_RIGHT_UPPER_MASS,BIPE_RIGHT_LOWER_MASS,BIPE_RIGHT_WHEEL_MASS};
static const double centers[7][2]={{BIPE_CHASSIS_X,BIPE_CHASSIS_Z},{BIPE_LEFT_UPPER_X,BIPE_LEFT_UPPER_Z},
    {BIPE_LEFT_LOWER_X,BIPE_LEFT_LOWER_Z},{BIPE_LEFT_WHEEL_X,BIPE_LEFT_WHEEL_Z},
    {BIPE_RIGHT_UPPER_X,BIPE_RIGHT_UPPER_Z},{BIPE_RIGHT_LOWER_X,BIPE_RIGHT_LOWER_Z},
    {BIPE_RIGHT_WHEEL_X,BIPE_RIGHT_WHEEL_Z}};
#else
/* The same 21 CAD bodies as the physical profile; indices within each side are
   crank0, input0, rotor0, rod0, crank1, input1, rotor1, rod1, wheel rotor, wheel. */
static const double masses[21]={BIPE_CHASSIS_MASS,
    BIPE_LEFT_CRANK0_MASS,BIPE_LEFT_HIPGEAR0_MASS,BIPE_LEFT_HIPROTOR0_MASS,BIPE_LEFT_ROD0_MASS,
    BIPE_LEFT_CRANK1_MASS,BIPE_LEFT_HIPGEAR1_MASS,BIPE_LEFT_HIPROTOR1_MASS,BIPE_LEFT_ROD1_MASS,
    BIPE_LEFT_ROTOR_MASS,BIPE_LEFT_WHEEL_MASS,
    BIPE_RIGHT_CRANK0_MASS,BIPE_RIGHT_HIPGEAR0_MASS,BIPE_RIGHT_HIPROTOR0_MASS,BIPE_RIGHT_ROD0_MASS,
    BIPE_RIGHT_CRANK1_MASS,BIPE_RIGHT_HIPGEAR1_MASS,BIPE_RIGHT_HIPROTOR1_MASS,BIPE_RIGHT_ROD1_MASS,
    BIPE_RIGHT_ROTOR_MASS,BIPE_RIGHT_WHEEL_MASS};
static const double centers[21][2]={{BIPE_CHASSIS_X,BIPE_CHASSIS_Z},
    {BIPE_LEFT_CRANK0_X,BIPE_LEFT_CRANK0_Z},{BIPE_LEFT_HIPGEAR0_X,BIPE_LEFT_HIPGEAR0_Z},
    {BIPE_LEFT_HIPROTOR0_X,BIPE_LEFT_HIPROTOR0_Z},{BIPE_LEFT_ROD0_X,BIPE_LEFT_ROD0_Z},
    {BIPE_LEFT_CRANK1_X,BIPE_LEFT_CRANK1_Z},{BIPE_LEFT_HIPGEAR1_X,BIPE_LEFT_HIPGEAR1_Z},
    {BIPE_LEFT_HIPROTOR1_X,BIPE_LEFT_HIPROTOR1_Z},{BIPE_LEFT_ROD1_X,BIPE_LEFT_ROD1_Z},
    {BIPE_LEFT_ROTOR_X,BIPE_LEFT_ROTOR_Z},{BIPE_LEFT_WHEEL_X,BIPE_LEFT_WHEEL_Z},
    {BIPE_RIGHT_CRANK0_X,BIPE_RIGHT_CRANK0_Z},{BIPE_RIGHT_HIPGEAR0_X,BIPE_RIGHT_HIPGEAR0_Z},
    {BIPE_RIGHT_HIPROTOR0_X,BIPE_RIGHT_HIPROTOR0_Z},{BIPE_RIGHT_ROD0_X,BIPE_RIGHT_ROD0_Z},
    {BIPE_RIGHT_CRANK1_X,BIPE_RIGHT_CRANK1_Z},{BIPE_RIGHT_HIPGEAR1_X,BIPE_RIGHT_HIPGEAR1_Z},
    {BIPE_RIGHT_HIPROTOR1_X,BIPE_RIGHT_HIPROTOR1_Z},{BIPE_RIGHT_ROD1_X,BIPE_RIGHT_ROD1_Z},
    {BIPE_RIGHT_ROTOR_X,BIPE_RIGHT_ROTOR_Z},{BIPE_RIGHT_WHEEL_X,BIPE_RIGHT_WHEEL_Z}};
#endif
static bool all_finite(const double *v,unsigned n) {
    for(unsigned j=0;j<n;j++) if(!isfinite(v[j])) return false;
    return true;
}
bool bipe_joint_limits(const double q[4],double tolerance) {
    if(!q || !all_finite(q,4) || !isfinite(tolerance) || tolerance<0 || tolerance>0.1) return false;
    for(unsigned j=0;j<4;j++) {
        double lo=j%2==0?BIPE_HIP_MIN:BIPE_KNEE_MIN,hi=j%2==0?BIPE_HIP_MAX:BIPE_KNEE_MAX;
        if(q[j]<lo-tolerance || q[j]>hi+tolerance) return false;
    } return true;
}
bool bipe_map_joints(const double raw[4],const double sign[4],const double zero[4],double mapped[4]) {
    if(!raw || !sign || !zero || !mapped) return false;
    memset(mapped,0,4*sizeof(double));
    if(!all_finite(raw,4) || !all_finite(sign,4) || !all_finite(zero,4)) return false;
    double result[4];
    for(unsigned j=0;j<4;j++) { if(fabs(sign[j])!=1) return false; result[j]=sign[j]*(raw[j]-zero[j]); }
    if(!bipe_joint_limits(result,0.05)) return false;
    memcpy(mapped,result,sizeof(result)); return true;
}
#if BIPE_SERIAL
/* Serial leg, q=[a1,a2] per side: thigh and knee-drive (shin) angles in the CAD
   convention, from body +x towards +z, i.e. positive about body -Y. */
static void rot(double a,const double v[2],double result[2]) {
    result[0]=cos(a)*v[0]-sin(a)*v[1]; result[1]=sin(a)*v[0]+cos(a)*v[1];
}
static void derivative(double a,const double v[2],double r[2]) {
    r[0]=-sin(a)*v[0]-cos(a)*v[1]; r[1]=cos(a)*v[0]-sin(a)*v[1];
}
static void serial_foot(const double q[2],double foot[2],double jac[2][2]) {
    foot[0]=BIPE_SERIAL_THIGH*cos(q[0])+BIPE_SERIAL_SHIN*cos(q[1]);
    foot[1]=BIPE_SERIAL_THIGH*sin(q[0])+BIPE_SERIAL_SHIN*sin(q[1]);
    jac[0][0]=-BIPE_SERIAL_THIGH*sin(q[0]); jac[1][0]=BIPE_SERIAL_THIGH*cos(q[0]);
    jac[0][1]=-BIPE_SERIAL_SHIN*sin(q[1]); jac[1][1]=BIPE_SERIAL_SHIN*cos(q[1]);
}
/* COM of one side's body (role 0..5, see masses) and its Jacobian d(x,z)/d(a1,a2). */
static void serial_body(const double q[2],unsigned role,const double center[2],double com[2],
                        double jac[2][2]) {
    double foot[2],fj[2][2],local[2],delta[2];
    serial_foot(q,foot,fj); memset(jac,0,4*sizeof(double));
    const double knee[2]={BIPE_SERIAL_THIGH*cos(q[0]),BIPE_SERIAL_THIGH*sin(q[0])};
    if(role==3) {
        for(unsigned k=0;k<2;k++) { com[k]=foot[k]+center[k]; jac[k][0]=fj[k][0]; jac[k][1]=fj[k][1]; }
        return;
    }
    unsigned joint=(role==0 || role==4)?0u:1u;
    double ratio=role>=4?BIPE_SERIAL_HIP_RATIO:1;
    rot(ratio*q[joint],center,local); derivative(ratio*q[joint],center,delta);
    const double rotor[2]={BIPE_SERIAL_MOTOR_X,BIPE_SERIAL_MOTOR_Z};
    for(unsigned k=0;k<2;k++) {
        double origin=role==2?knee[k]:(role>=4?rotor[k]:0);
        com[k]=origin+local[k]; jac[k][joint]=ratio*delta[k];
        if(role==2) jac[k][0]=fj[k][0];
    }
}
static bool geometry(const double q[4],double com[2],double axle[2]) {
    for(unsigned k=0;k<2;k++) { com[k]=masses[0]*centers[0][k]; axle[k]=0; }
    for(unsigned side=0;side<2;side++) {
        double foot[2],fj[2][2]; serial_foot(q+2*side,foot,fj);
        for(unsigned role=0;role<6;role++) {
            unsigned i=1+6*side+role; double one[2],jac[2][2];
            serial_body(q+2*side,role,centers[i],one,jac);
            for(unsigned k=0;k<2;k++) com[k]+=masses[i]*one[k];
        }
        for(unsigned k=0;k<2;k++) axle[k]+=foot[k]/2;
    }
    for(unsigned k=0;k<2;k++) com[k]/=BIPE_TOTAL_MASS;
    return true;
}
/* The wheel encoder reads the wheel against the shin, whose body-frame angle is a2. */
bool bipe_carrier_rates(const double q[4],const double qd[4],double rates[2]) {
    if(!q || !qd || !rates || !all_finite(q,4) || !all_finite(qd,4)) return false;
    for(unsigned side=0;side<2;side++) rates[side]=qd[2*side+1];
    return true;
}
#elif !BIPE_FIVEBAR
static void rot(double a,const double v[2],double result[2]) {
    result[0]=cos(a)*v[0]+sin(a)*v[1]; result[1]=-sin(a)*v[0]+cos(a)*v[1];
}
static bool geometry(const double q[4],double com[2],double axle[2]) {
    for(unsigned k=0;k<2;k++) { com[k]=masses[0]*centers[0][k]; axle[k]=0; }
    for(unsigned side=0;side<2;side++) {
        unsigned i=1+side*3; double a=q[2*side],b=q[2*side+1],u[2],lo[2];
        double knee[2]={-BIPE_LINK_LENGTH*sin(a),-BIPE_LINK_LENGTH*cos(a)};
        double foot[2]={knee[0]-BIPE_LINK_LENGTH*sin(a+b),knee[1]-BIPE_LINK_LENGTH*cos(a+b)};
        rot(a,centers[i],u); rot(a+b,centers[i+1],lo);
        for(unsigned k=0;k<2;k++) {
            com[k]+=masses[i]*u[k]+masses[i+1]*(knee[k]+lo[k])+masses[i+2]*(foot[k]+centers[i+2][k]);
            axle[k]+=foot[k]/2;
        }
    } for(unsigned k=0;k<2;k++) com[k]/=BIPE_TOTAL_MASS;
    return true;
}
bool bipe_carrier_rates(const double q[4],const double qd[4],double rates[2]) {
    if(!q || !qd || !rates || !all_finite(q,4) || !all_finite(qd,4)) return false;
    for(unsigned side=0;side<2;side++) rates[side]=-qd[2*side]-qd[2*side+1];
    return true;
}
#else
typedef struct {
    double elbow[2][2], elbow_derivative[2][2], foot[2], jac[2][2];
    double theta[2], theta_jac[2][2];
} fivebar_geometry;
static void rot(double a,const double v[2],double result[2]) {
    result[0]=cos(a)*v[0]-sin(a)*v[1]; result[1]=sin(a)*v[0]+cos(a)*v[1];
}
static void derivative(double a,const double v[2],double r[2]) {
    r[0]=-sin(a)*v[0]-cos(a)*v[1]; r[1]=cos(a)*v[0]-sin(a)*v[1];
}
static bool fivebar(const double q[2],fivebar_geometry *g) {
    for(unsigned i=0;i<2;i++) {
        g->elbow[i][0]=(i==0?-0.5:0.5)*BIPE_FIVEBAR_BASE+BIPE_FIVEBAR_CRANK*cos(q[i]);
        g->elbow[i][1]=BIPE_FIVEBAR_CRANK*sin(q[i]);
        g->elbow_derivative[i][0]=-BIPE_FIVEBAR_CRANK*sin(q[i]);
        g->elbow_derivative[i][1]=BIPE_FIVEBAR_CRANK*cos(q[i]);
    }
    double dx=g->elbow[1][0]-g->elbow[0][0],dz=g->elbow[1][1]-g->elbow[0][1];
    double length=hypot(dx,dz),h2=BIPE_FIVEBAR_ROD*BIPE_FIVEBAR_ROD-length*length/4;
    if(length<1e-9 || h2<=0) return false;
    double h=(dx<0?-1:1)*sqrt(h2);
    g->foot[0]=(g->elbow[0][0]+g->elbow[1][0])/2+h*dz/length;
    g->foot[1]=(g->elbow[0][1]+g->elbow[1][1])/2-h*dx/length;
    double u[2][2];
    for(unsigned i=0;i<2;i++) for(unsigned k=0;k<2;k++) u[i][k]=g->foot[k]-g->elbow[i][k];
    double det=u[0][0]*u[1][1]-u[0][1]*u[1][0];
    if(fabs(det)<1e-10) return false;
    for(unsigned j=0;j<2;j++) {
        double rhs=u[j][0]*g->elbow_derivative[j][0]+u[j][1]*g->elbow_derivative[j][1];
        g->jac[0][j]=(j==0?u[1][1]:-u[0][1])*rhs/det;
        g->jac[1][j]=(j==0?-u[1][0]:u[0][0])*rhs/det;
    }
    for(unsigned i=0;i<2;i++) {
        g->theta[i]=atan2(u[i][1],u[i][0]);
        for(unsigned j=0;j<2;j++) {
            double vx=g->jac[0][j]-(i==j?g->elbow_derivative[i][0]:0);
            double vz=g->jac[1][j]-(i==j?g->elbow_derivative[i][1]:0);
            g->theta_jac[i][j]=(u[i][0]*vz-u[i][1]*vx)/(BIPE_FIVEBAR_ROD*BIPE_FIVEBAR_ROD);
        }
    } return true;
}
/* Origins, orientations and COM Jacobians use the CAD angle convention, whose
   positive rotation is opposite positive body pitch about robot +Y. */
static void body_geometry(const double q[2],const fivebar_geometry *g,unsigned role,
                          const double center[2],double com[2],double jac[2][2]) {
    memset(jac,0,4*sizeof(double));
    double origin[2]={0,0},angle=0,angle_jac[2]={0,0},local[2],delta[2];
    if(role<8) {
        unsigned hinge=role/4,kind=role%4;
        origin[0]=(hinge==0?-0.5:0.5)*BIPE_FIVEBAR_BASE;
        if(kind==3) {
            memcpy(origin,g->elbow[hinge],sizeof(origin)); angle=g->theta[hinge];
            memcpy(angle_jac,g->theta_jac[hinge],sizeof(angle_jac));
            for(unsigned k=0;k<2;k++) jac[k][hinge]=g->elbow_derivative[hinge][k];
        } else {
            double ratio=kind==2?BIPE_FIVEBAR_HIP_RATIO:1;
            angle=ratio*q[hinge]; angle_jac[hinge]=ratio;
            if(kind==2) origin[1]=BIPE_FIVEBAR_HIP_MOTOR_Z;
        }
    } else if(role==8) {
        const double motor[2]={BIPE_FIVEBAR_WHEEL_MOTOR_X,BIPE_FIVEBAR_WHEEL_MOTOR_Z};
        rot(g->theta[0],motor,local); derivative(g->theta[0],motor,delta);
        for(unsigned k=0;k<2;k++) {
            origin[k]=g->elbow[0][k]+local[k];
            for(unsigned j=0;j<2;j++) jac[k][j]=(j==0?g->elbow_derivative[0][k]:0)+delta[k]*g->theta_jac[0][j];
        }
        angle=(1-BIPE_FIVEBAR_WHEEL_RATIO)*g->theta[0];
        for(unsigned j=0;j<2;j++) angle_jac[j]=(1-BIPE_FIVEBAR_WHEEL_RATIO)*g->theta_jac[0][j];
    } else {
        memcpy(origin,g->foot,sizeof(origin)); memcpy(jac,g->jac,sizeof(g->jac));
    }
    rot(angle,center,local); derivative(angle,center,delta);
    for(unsigned k=0;k<2;k++) {
        com[k]=origin[k]+local[k];
        for(unsigned j=0;j<2;j++) jac[k][j]+=delta[k]*angle_jac[j];
    }
}
static bool geometry(const double q[4],double com[2],double axle[2]) {
    for(unsigned k=0;k<2;k++) { com[k]=masses[0]*centers[0][k]; axle[k]=0; }
    for(unsigned side=0;side<2;side++) {
        fivebar_geometry g;
        if(!fivebar(q+2*side,&g)) return false;
        for(unsigned role=0;role<10;role++) {
            unsigned i=1+10*side+role; double one[2],jac[2][2];
            body_geometry(q+2*side,&g,role,centers[i],one,jac);
            for(unsigned k=0;k<2;k++) com[k]+=masses[i]*one[k];
        }
        for(unsigned k=0;k<2;k++) axle[k]+=g.foot[k]/2;
    }
    for(unsigned k=0;k<2;k++) com[k]/=BIPE_TOTAL_MASS;
    return true;
}
bool bipe_carrier_rates(const double q[4],const double qd[4],double rates[2]) {
    if(!q || !qd || !rates || !all_finite(q,4) || !all_finite(qd,4)) return false;
    for(unsigned side=0;side<2;side++) {
        fivebar_geometry g;
        if(!fivebar(q+2*side,&g)) return false;
        rates[side]=g.theta_jac[0][0]*qd[2*side]+g.theta_jac[0][1]*qd[2*side+1];
    } return true;
}
#endif
bool bipe_posture(double angle_deg,double q[4]) {
    if(!q || !isfinite(angle_deg)) return false;
#if BIPE_SERIAL
    if(angle_deg<30 || angle_deg>65) return false;
    const double commands[4]={30,40,60,65};
    const double heights[4]={BIPE_SERIAL_HEIGHT0,BIPE_SERIAL_HEIGHT1,BIPE_SERIAL_HEIGHT2,BIPE_SERIAL_HEIGHT3};
    const double xs[4]={BIPE_SERIAL_AXLE_X0,BIPE_SERIAL_AXLE_X1,BIPE_SERIAL_AXLE_X2,BIPE_SERIAL_AXLE_X3};
    unsigned interval=angle_deg<40?0u:(angle_deg<60?1u:2u);
    double u=(angle_deg-commands[interval])/(commands[interval+1]-commands[interval]),s=u*u*(3-2*u);
    double x=xs[interval]+(xs[interval+1]-xs[interval])*s;
    double z=-(heights[interval]+(heights[interval+1]-heights[interval])*s);
    double l1=BIPE_SERIAL_THIGH,l2=BIPE_SERIAL_SHIN,d=hypot(x,z);
    if(!(fabs(l1-l2)<d && d<l1+l2)) return false;
    double along=(l1*l1-l2*l2+d*d)/(2*d),across=sqrt(l1*l1-along*along),ux=x/d,uz=z/d;
    /* Knee behind the hip-axle line: the intersection whose offset from it points to -x. */
    double best[2]={0,0},score=INFINITY;
    for(int sign=1;sign>=-1;sign-=2) {
        double k[2]={along*ux-sign*across*uz,along*uz+sign*across*ux};
        double offset=k[0]-(k[0]*ux+k[1]*uz)*ux;
        if(offset<score) { score=offset; best[0]=k[0]; best[1]=k[1]; }
    }
    q[0]=atan2(best[1],best[0]); if(q[0]>0) q[0]-=6.28318530717958647692;
    q[1]=atan2(z-best[1],x-best[0]);
#elif BIPE_FIVEBAR
    if(angle_deg<30 || angle_deg>65) return false;
    const double commands[4]={30,40,60,65};
    const double heights[4]={BIPE_FIVEBAR_MAX_HEIGHT,BIPE_FIVEBAR_NOMINAL_HEIGHT,
        BIPE_FIVEBAR_CROUCH_HEIGHT,BIPE_FIVEBAR_MIN_HEIGHT};
    unsigned interval=angle_deg<40?0u:(angle_deg<60?1u:2u);
    double u=(angle_deg-commands[interval])/(commands[interval+1]-commands[interval]);
    double height=heights[interval]+(heights[interval+1]-heights[interval])*u*u*(3-2*u);
    double radius=hypot(BIPE_FIVEBAR_BASE/2,height);
    double cosine=(radius*radius+BIPE_FIVEBAR_CRANK*BIPE_FIVEBAR_CRANK-
        BIPE_FIVEBAR_ROD*BIPE_FIVEBAR_ROD)/(2*radius*BIPE_FIVEBAR_CRANK);
    if(fabs(cosine)>1) return false;
    double bend=acos(cosine);
    q[0]=atan2(-height,BIPE_FIVEBAR_BASE/2)-bend;
    q[1]=atan2(-height,-BIPE_FIVEBAR_BASE/2)+bend;
#else
    q[0]=angle_deg*0.017453292519943295; q[1]=-2*q[0];
#endif
    q[2]=q[0]; q[3]=q[1]; return true;
}
bool bipe_kinematics(const double q[4],double com[2],double axle[2],double *eq,double *scale) {
    if(!q || !com || !axle || !eq || !scale || !bipe_joint_limits(q,0.05)) return false;
    if(!geometry(q,com,axle)) return false;
    *eq=-atan2(com[0]-axle[0],com[1]-axle[1]);
#if BIPE_SERIAL
    double nominal[4]; if(!bipe_posture(40,nominal)) return false;
#elif BIPE_FIVEBAR
    const double nominal[4]={BIPE_FIVEBAR_NOMINAL_Q0,BIPE_FIVEBAR_NOMINAL_Q1,
        BIPE_FIVEBAR_NOMINAL_Q0,BIPE_FIVEBAR_NOMINAL_Q1};
#else
    const double angle=0.6981317007977318,nominal[4]={angle,-2*angle,angle,-2*angle};
#endif
    double nc[2],na[2]; if(!geometry(nominal,nc,na)) return false;
    *scale=BIPE_POSTURE_GAIN_SCHEDULE?(com[1]-axle[1])/(nc[1]-na[1]):1; return true;
}
#if !BIPE_FIVEBAR && !BIPE_SERIAL
static void derivative(double a,const double v[2],double r[2]) {
    r[0]=-sin(a)*v[0]+cos(a)*v[1]; r[1]=-cos(a)*v[0]-sin(a)*v[1];
}
#endif
bool bipe_feedforward(const double q[4],double pitch,double roll,double torque[4]) {
    if(!torque) return false;
    memset(torque,0,4*sizeof(double));
    if(!bipe_joint_limits(q,0.05) || !isfinite(pitch) || !isfinite(roll)) return false;
    double up[2]={-sin(pitch),cos(pitch)*cos(roll)};
    for(unsigned side=0;side<2;side++) {
#if BIPE_SERIAL
        double weighted[2][2]={{0,0},{0,0}},foot[2],fj[2][2];
        serial_foot(q+2*side,foot,fj);
        for(unsigned role=0;role<6;role++) {
            unsigned i=1+6*side+role; double com[2],jac[2][2];
            serial_body(q+2*side,role,centers[i],com,jac);
            for(unsigned k=0;k<2;k++) for(unsigned j=0;j<2;j++) weighted[k][j]+=masses[i]*jac[k][j];
        }
        for(unsigned j=0;j<2;j++) for(unsigned k=0;k<2;k++)
            torque[2*side+j]+=BIPE_GRAVITY*(weighted[k][j]-fj[k][j]*BIPE_TOTAL_MASS/2)*up[k];
#elif BIPE_FIVEBAR
        fivebar_geometry g; double weighted[2][2]={{0,0},{0,0}};
        if(!fivebar(q+2*side,&g)) return false;
        for(unsigned role=0;role<10;role++) {
            unsigned i=1+10*side+role; double com[2],jac[2][2];
            body_geometry(q+2*side,&g,role,centers[i],com,jac);
            for(unsigned k=0;k<2;k++) for(unsigned j=0;j<2;j++) weighted[k][j]+=masses[i]*jac[k][j];
        }
        for(unsigned j=0;j<2;j++) for(unsigned k=0;k<2;k++)
            torque[2*side+j]+=BIPE_GRAVITY*(weighted[k][j]-g.jac[k][j]*BIPE_TOTAL_MASS/2)*up[k];
#else
        unsigned i=1+side*3; double a=q[2*side],b=q[2*side+1],u[2],lo[2];
        derivative(a,centers[i],u); derivative(a+b,centers[i+1],lo);
        double jk[2]={-BIPE_LINK_LENGTH*cos(a),BIPE_LINK_LENGTH*sin(a)};
        double distal[2]={-BIPE_LINK_LENGTH*cos(a+b),BIPE_LINK_LENGTH*sin(a+b)};
        for(unsigned j=0;j<2;j++) for(unsigned k=0;k<2;k++) {
            double jac=distal[k]+(j==0?jk[k]:0);
            double weight=masses[i]*(j==0?u[k]:0)+masses[i+1]*(lo[k]+(j==0?jk[k]:0))+masses[i+2]*jac;
            torque[2*side+j]+=BIPE_GRAVITY*(weight-jac*BIPE_TOTAL_MASS/2)*up[k];
        }
#endif
    } return true;
}
bool bipe_legs(const double q[4],const double qd[4],const double qr[4],const double qdr[4],
    const double ff[4],double torque[4]) {
    if(!torque) return false;
    memset(torque,0,4*sizeof(double));
    if(!bipe_joint_limits(q,0.05) || !bipe_joint_limits(qr,0) || !qd || !qdr || !ff ||
       !all_finite(qd,4) || !all_finite(qdr,4) || !all_finite(ff,4)) return false;
    for(unsigned j=0;j<4;j++) torque[j]=fmax(-BIPE_JOINT_PEAK,fmin(BIPE_JOINT_PEAK,
        ff[j]+BIPE_JOINT_KP*(qr[j]-q[j])+BIPE_JOINT_KD*(qdr[j]-qd[j]))); return true;
}
bool bipe_control_step(bipe_balance *s,const bipe_estimate *e,double velocity,double yaw_rate,
    const double qr[4],const double qdr[4],double dt,double torque[6],double ff[4],double *reference) {
    if(!s || !e || !torque || !ff || !reference) return false;
    memset(torque,0,6*sizeof(double)); memset(ff,0,4*sizeof(double)); *reference=0;
    double com[2],axle[2],eq,scale;
    if(!bipe_kinematics(e->q,com,axle,&eq,&scale) || !bipe_feedforward(e->q,e->pitch,e->roll,ff) ||
       !bipe_legs(e->q,e->qd,qr,qdr,ff,torque)) return false;
    bipe_observation o={e->pitch,e->pitch_rate,e->yaw,e->yaw_rate,e->x,e->y,e->velocity,eq,scale,
        {e->wheel_speed[0],e->wheel_speed[1]}, {0}, {0}, {0}, {0}};
    memcpy(o.q,e->q,sizeof(o.q)); memcpy(o.qd,e->qd,sizeof(o.qd));
    memcpy(o.qref,qr,sizeof(o.qref)); memcpy(o.qdref,qdr,sizeof(o.qdref));
    if(!bipe_balance_step(s,&o,velocity,yaw_rate,dt,torque+4)) { memset(torque,0,6*sizeof(double)); return false; }
#if BIPE_SERIAL
    if(BIPE_KNEE_WHEEL_REACTION!=0) {
        /* The hub's stator sits in the shin, whose angle the 1:1 knee belt holds from the knee
           drive: the wheel torque reacts on the knee drive. Its axis is opposite the wheel's
           (knee -y, wheel +y), so the knee takes -wheel torque to cancel it. */
        for(unsigned side=0;side<2;side++) ff[2*side+1]-=BIPE_KNEE_WHEEL_REACTION*torque[4+side];
        if(!bipe_legs(e->q,e->qd,qr,qdr,ff,torque)) { memset(torque,0,6*sizeof(double)); return false; }
    }
#endif
    *reference=eq;
#if !BIPE_BALANCE_LQR
    double speed_kp=BIPE_SPEED_KP, speed_ki=BIPE_SPEED_KI;
#if BIPE_FIVEBAR || BIPE_SERIAL
    if(velocity==0 && !s->braking) { speed_kp=BIPE_HOLD_SPEED_KP; speed_ki=BIPE_HOLD_SPEED_KI; }
#endif
    *reference+=s->slope_lean+s->acceleration_lean+fmax(-BIPE_MAX_PITCH_OFFSET,fmin(BIPE_MAX_PITCH_OFFSET,
        speed_kp*(s->vref-e->velocity)+speed_ki*s->integral));
#endif
    return true;
}

/* Estimator-only protective crouch, with a bounded local torque-off deadline. */
static double fall_clamp(double x,double lo,double hi) { return fmax(lo,fmin(hi,x)); }
static bool fall_config_valid(const bipe_fall_settings *c) {
    if(!c) return false;
    const double a[]={c->pitch_rad,c->hard_pitch_rad,c->roll_rad,c->outward_rate_rad_s,
        c->persist_s,c->freefall_m_s2,c->freefall_persist_s,c->posture_deg,c->reference_rate_rad_s,
        c->crouch_s,c->ramp_s,c->wheel_ramp_s,c->joint_cap_Nm};
    for(unsigned j=0;j<sizeof(a)/sizeof(a[0]);j++) if(!isfinite(a[j]) || a[j]<=0) return false;
    return c->hard_pitch_rad>=c->pitch_rad && c->posture_deg>=30 && c->posture_deg<=65 &&
        c->crouch_s+c->ramp_s<=0.5 && c->joint_cap_Nm<=BIPE_JOINT_PEAK;
}
bool bipe_fall_legs(const bipe_fall *s,const bipe_fall_settings *c,const double q[4],
                    const double qd[4],double torque[4]) {
    if(!s || !c || !q || !qd || !torque || !fall_config_valid(c)) return false;
    memset(torque,0,4*sizeof(double));
    if(!s->latched || s->torque_off) return true;
    if(!all_finite(q,4) || !all_finite(qd,4)) return false;
    double scale=fall_clamp((c->crouch_s+c->ramp_s-s->age_s)/c->ramp_s,0,1);
    for(unsigned j=0;j<4;j++) torque[j]=scale*fall_clamp(BIPE_JOINT_KP*(s->qref[j]-q[j])+
        BIPE_JOINT_KD*(s->qdref[j]-qd[j]),-c->joint_cap_Nm,c->joint_cap_Nm);
    return true;
}
bool bipe_fall_step(bipe_fall *s,const bipe_fall_settings *c,const bipe_estimate *e,
                    bool valid,double dt,const double previous[6],double torque[6]) {
    if(!s || !c || !e || !previous || !torque || !fall_config_valid(c) ||
       !isfinite(dt) || dt<=0 || dt>0.02) return false;
    memset(torque,0,6*sizeof(double));
    if(s->torque_off) return true;
    double com[2],axle[2],eq=0,scale=0;
    const double observed[]={e->pitch,e->pitch_rate,e->roll,e->velocity,e->accel_norm};
    valid=valid && all_finite(observed,5) && e->accel_norm>=0 && all_finite(e->q,4) &&
        all_finite(e->qd,4) && all_finite(previous,6) &&
        bipe_kinematics(e->q,com,axle,&eq,&scale);
    if(!valid) { s->latched=true; s->torque_off=true; s->reason=BIPE_FALL_INVALID; return true; }
    if(!s->latched) {
        double error=e->pitch-eq;
        bool pitch=fabs(error)>=c->pitch_rad && error*e->pitch_rate>
            fabs(error)*c->outward_rate_rad_s;
        bool roll=fabs(e->roll)>=c->roll_rad;
        s->exposure_s=(pitch || roll)?s->exposure_s+dt:0;
        s->freefall_s=e->accel_norm<c->freefall_m_s2?s->freefall_s+dt:0;
        bool hard=fabs(error)>=c->hard_pitch_rad;
        if(!hard && s->exposure_s+1e-12<c->persist_s &&
           s->freefall_s+1e-12<c->freefall_persist_s) return true;
        s->latched=true; s->age_s=0;
        s->reason=s->freefall_s+1e-12>=c->freefall_persist_s?BIPE_FALL_FREEFALL:
            (roll?BIPE_FALL_ROLL:BIPE_FALL_PITCH);
        memcpy(s->qref,e->q,sizeof(s->qref)); memcpy(s->wheel,previous+4,sizeof(s->wheel));
    } else s->age_s+=dt;
    if(s->age_s+1e-12>=c->crouch_s+c->ramp_s) {
        s->torque_off=true; memset(s->qdref,0,sizeof(s->qdref)); return true;
    }
    double target[4]; if(!bipe_posture(c->posture_deg,target)) return false;
    for(unsigned j=0;j<4;j++) {
        double change=fall_clamp(target[j]-s->qref[j],-c->reference_rate_rad_s*dt,
            c->reference_rate_rad_s*dt);
        s->qref[j]+=change; s->qdref[j]=change/dt;
    }
    if(!bipe_fall_legs(s,c,e->q,e->qd,torque)) return false;
    double wheel_scale=fmax(0,1-s->age_s/c->wheel_ramp_s);
    for(unsigned j=0;j<2;j++) torque[4+j]=wheel_scale*s->wheel[j];
    return true;
}
