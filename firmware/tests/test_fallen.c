#include "fallen.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
/* M17: the fallen mode hands over to the balance estimator only after
   BIPE_FALLEN_TAKEOVER_TICKS quiet ticks in a row, and keeps it once it has. */
static bool step(bipe_fallen_estimator *s,const bipe_sensor_config *c,const double g[3],
    const double a[3],const double enc[6]) {
    bipe_estimate out; bool valid=false;
    assert(bipe_fallen_step(s,c,g,a,enc,&out,&valid));
    return valid;
}
int main(void) {
    double enc[6]={0.6981317007977318,-1.3962634015954636,0.6981317007977318,-1.3962634015954636,0,0};
    double still[3]={0.003,-0.002,0.001},spin[3]={0,2.0,0},a[3]={0,0,9.81},hard[3]={0,0,15.7};
    bipe_sensor_config c=bipe_default_sensor_config();
    bipe_fallen_estimator s; memset(&s,0,sizeof(s));
    /* Spinning or at 1.6 g: never quiet, never takes over. */
    for(unsigned i=0;i<100;i++) assert(!step(&s,&c,spin,a,enc));
    for(unsigned i=0;i<100;i++) assert(!step(&s,&c,still,hard,enc));
    /* A non-quiet tick resets the run. */
    for(unsigned i=0;i<BIPE_FALLEN_TAKEOVER_TICKS-1;i++) assert(!step(&s,&c,still,a,enc));
    assert(!step(&s,&c,spin,a,enc));
    /* 19 quiet ticks do not take over (the first tick re-initialises), the 20th does. */
    for(unsigned i=0;i<BIPE_FALLEN_TAKEOVER_TICKS-1;i++) assert(!step(&s,&c,still,a,enc));
    assert(step(&s,&c,still,a,enc)); assert(s.accepted==1);
    /* After the takeover, motion keeps the estimator running. */
    assert(step(&s,&c,spin,a,enc)); assert(s.accepted==2);
    /* An estimator refusal (here an encoder jump) hands back to the raw channels. The quiet
       run is about the packets, not the estimator: unbroken, the next tick takes over again;
       broken by motion, a fresh run of 20 is needed. */
    for(unsigned i=0;i<BIPE_FALLEN_TAKEOVER_TICKS;i++) assert(step(&s,&c,still,a,enc));
    enc[4]+=10; assert(!step(&s,&c,still,a,enc)); assert(s.accepted==0);
    assert(step(&s,&c,still,a,enc));
    enc[4]+=10; assert(!step(&s,&c,spin,a,enc)); assert(s.accepted==0);
    for(unsigned i=0;i<BIPE_FALLEN_TAKEOVER_TICKS-1;i++) assert(!step(&s,&c,still,a,enc));
    assert(step(&s,&c,still,a,enc));
    puts("fallen: quiet takeover after 20 ticks, reset by motion or 1.6 g, kept once taken, refusal hands back");
    return 0;
}
