/* Permission composition is part of the portable application, independently
   exercised with synthetic telemetry. Physical adapters require commissioning. */
#include "runtime.h"
#include <string.h>
static bool send_frame(void *context, const bipe_frame *frame) {
    bipe_runtime *r=context;
    if(!bipe_policy_power_permission(&r->policy) ||
       (r->policy.state!=BIPE_POLICY_RUN && r->policy.state!=BIPE_POLICY_STOP)) return false;
    r->disabled=false;
    return r->transport.send && r->transport.send(r->transport.context,frame);
}
static bool disable(void *context) {
    bipe_runtime *r=context;
    r->disabled=r->transport.disable_and_cancel_pending &&
        r->transport.disable_and_cancel_pending(r->transport.context);
    return r->disabled;
}
static void power(void *context, bool permission, bool heartbeat) {
    bipe_runtime *r=context;
    if(r->transport.power) r->transport.power(r->transport.context,
        permission && bipe_policy_power_permission(&r->policy),heartbeat);
}
void bipe_runtime_init(bipe_runtime *r,const bipe_app_profile *profile,
                       bipe_app_hal hal,uint32_t evidence,uint32_t now) {
    memset(r,0,sizeof(*r)); r->transport=hal;
    bipe_policy_init(&r->policy,now,profile && profile->synthetic,evidence);
    bipe_app_init(&r->application,profile,(bipe_app_hal){send_frame,power,r,disable});
}
bool bipe_runtime_begin(bipe_runtime *r,uint32_t now) {
    if(!bipe_app_profile_authorized(&r->application) || !r->transport.send || !r->transport.power || !r->transport.disable_and_cancel_pending || r->application.supervisor.state!=BIPE_OFF) return false;
    return bipe_policy_begin(&r->policy,now);
}
void bipe_runtime_stop(bipe_runtime *r,uint32_t now) {
    bipe_policy_request_stop(&r->policy,now);
    if(r->application.supervisor.state==BIPE_ACTIVE) {
        r->application.supervisor.state=BIPE_STOPPING;
        r->application.supervisor.stop_since=now;
    }
}
void bipe_runtime_tick(bipe_runtime *r,const bipe_policy_sample *sample,uint32_t now) {
    if(sample) (void)bipe_policy_sample_update(&r->policy,sample,now);
    bipe_policy_tick(&r->policy,now);
    if(bipe_policy_power_permission(&r->policy) && !bipe_app_profile_authorized(&r->application)) {
        bipe_policy_reset(&r->policy,now);power(r,false,false);
        if(r->application.commands_started) { (void)disable(r);r->application.commands_started=false; }
        return;
    }
    if(r->policy.state==BIPE_POLICY_START) {
        /* Startup heartbeat is finite and sensing supervised; no commands. */
        if(now-r->last_heartbeat_ms>=10u) { r->heartbeat=!r->heartbeat; r->last_heartbeat_ms=now; }
        power(r,true,r->heartbeat); return;
    }
    if(r->policy.state==BIPE_POLICY_RUN && r->application.supervisor.state==BIPE_OFF)
        bipe_start(&r->application.supervisor);
    if(r->policy.state==BIPE_POLICY_STOP) {
        bipe_runtime_stop(r,now);
        if(!r->application.commands_started && r->application.supervisor.state!=BIPE_ACTIVE &&
           r->application.supervisor.state!=BIPE_STOPPING) {
            if(disable(r)) (void)bipe_policy_stopped(&r->policy,true,now);
        }
    }
    if(!bipe_policy_power_permission(&r->policy)) {
        power(r,false,false);
        if(r->application.commands_started) {
            (void)disable(r);r->application.commands_started=false;
        }
        return;
    }
    bipe_app_tick(&r->application,now);
    if(r->application.supervisor.state==BIPE_STOPPING && r->policy.state==BIPE_POLICY_RUN)
        bipe_policy_request_stop(&r->policy,now);
    if(r->application.supervisor.state==BIPE_FAULT) {
        bipe_policy_reset(&r->policy,now);power(r,false,false);
    }
    if(r->policy.state==BIPE_POLICY_STOP && !r->application.commands_started && r->disabled) {
        (void)bipe_policy_stopped(&r->policy,true,now);power(r,false,false);
    }
}
void bipe_runtime_reset(bipe_runtime *r,uint32_t now) {
    bipe_policy_reset(&r->policy,now);
    bipe_app_reset(&r->application);power(r,false,false);
    r->heartbeat=false;
}
