#include "bipe.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int main(void) {
    double q[4]={0.6981317007977318,-1.3962634015954636,0.6981317007977318,-1.3962634015954636};
    double signs[4]={1,-1,1,-1},zeros[4]={0.2,-0.3,0.4,-0.1},raw[4],mapped[4];
    for(unsigned j=0;j<4;j++) raw[j]=q[j]/signs[j]+zeros[j];
    assert(bipe_map_joints(raw,signs,zeros,mapped));
    for(unsigned j=0;j<4;j++) assert(fabs(mapped[j]-q[j])<1e-14);
    signs[0]=0; assert(!bipe_map_joints(raw,signs,zeros,mapped)); signs[0]=1;
    zeros[1]+=3; assert(!bipe_map_joints(raw,signs,zeros,mapped));
    double qr[4]; memcpy(qr,q,sizeof(q)); qr[0]=2; assert(!bipe_joint_limits(qr,0));
    double zero[4]={0},ff[4],torque[6]={1,1,1,1,1,1};
    assert(bipe_feedforward(q,0,0,ff)); assert(bipe_legs(q,zero,q,zero,ff,torque));
    assert(!bipe_legs(q,zero,qr,zero,ff,torque));
    for(unsigned j=0;j<4;j++) assert(torque[j]==0);
    double g[3]={0.003,-0.002,0.001},a[3]={0,0,9.81},enc[6];
    memcpy(enc,q,sizeof(q)); enc[4]=enc[5]=0;
    bipe_estimator e; bipe_estimate out; bipe_sensor_config settings=bipe_default_sensor_config();
    assert(bipe_estimator_init(&e,a,enc)); assert(bipe_estimator_step(&e,&settings,g,a,enc,&out));
    g[0]=NAN; bipe_estimator old=e; assert(!bipe_estimator_step(&e,&settings,g,a,enc,&out));
    assert(memcmp(&old,&e,sizeof(e))==0); g[0]=0.003;
    enc[4]+=1; assert(!bipe_estimator_step(&e,&settings,g,a,enc,&out)); enc[4]-=1;
    settings.velocity_tau=0; assert(!bipe_estimator_step(&e,&settings,g,a,enc,&out));
    bipe_calibration cal={0}; double bias[3],gravity[3]; bool done=false;
    for(uint32_t ms=0;ms<=2000;ms++) done=bipe_calibrate(&cal,g,a,true,ms,ms,bias,gravity);
    assert(done); for(unsigned j=0;j<3;j++) assert(fabs(bias[j]-g[j])<1e-13);
    assert(!bipe_calibrate(&cal,g,a,true,2001,2000,bias,gravity));
    assert(!bipe_calibrate(&cal,g,a,false,2002,2002,bias,gravity)); assert(cal.samples==0);
    assert(!bipe_calibrate(&cal,g,a,true,3000,3000,bias,gravity));
    assert(!bipe_calibrate(&cal,g,a,true,5000,5000,bias,gravity)); assert(cal.samples==1);
    bipe_balance balance={0}; out=(bipe_estimate){0}; memcpy(out.q,q,sizeof(q)); double reference;
    assert(bipe_control_step(&balance,&out,0,0,q,zero,0.005,torque,ff,&reference));
    out.q[0]=NAN; assert(!bipe_control_step(&balance,&out,0,0,q,zero,0.005,torque,ff,&reference));
    for(unsigned j=0;j<6;j++) assert(torque[j]==0);
    puts("control: mapping/sign/zero limits, invalid-output inhibit, estimator rejection, stationary calibration pass");
    return 0;
}
