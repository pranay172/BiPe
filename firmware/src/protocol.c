#include "bipe.h"
#include <math.h>
#include <string.h>

static bool quantize(double x, bipe_range range, unsigned bits, uint32_t *result) {
    double span=range.hi-range.lo;
    if (!isfinite(x) || !isfinite(range.lo) || !isfinite(range.hi) ||
        !isfinite(span) || span<=0 || x < range.lo || x > range.hi) return false;
    /* Endpoints must fit their bit field. Manual's 2^bits example overflows at max. */
    *result = (uint32_t)(((x-range.lo)/span) * ((1u<<bits)-1u));
    return true;
}
bool bipe_pack(uint8_t id, const bipe_ranges *r, const bipe_mit *c, bipe_frame *f) {
    uint32_t kp, kd, p, v, t;
    if (!id || !r || !c || !f ||
        !quantize(c->kp,r->kp,12,&kp) || !quantize(c->kd,r->kd,12,&kd) ||
        !quantize(c->position,r->position,16,&p) ||
        !quantize(c->velocity,r->velocity,12,&v) ||
        !quantize(c->effort,r->effort,12,&t)) return false;
    memset(f,0,sizeof(*f)); f->id=0x800u|id; f->dlc=8; f->extended=true;
    f->data[0]=(uint8_t)(kp>>4); f->data[1]=(uint8_t)((kp<<4)|(kd>>8));
    f->data[2]=(uint8_t)kd; f->data[3]=(uint8_t)(p>>8); f->data[4]=(uint8_t)p;
    f->data[5]=(uint8_t)(v>>4); f->data[6]=(uint8_t)((v<<4)|(t>>8));
    f->data[7]=(uint8_t)t; return true;
}
static int32_t signed16(const uint8_t *p) {
    uint32_t value=((uint32_t)p[0]<<8)|p[1];
    return value<32768u ? (int32_t)value : (int32_t)value-65536;
}
bool bipe_decode(const bipe_frame *f, unsigned pairs, double reduction,
                 bipe_feedback *r) {
    if (!f || !r || !f->extended || f->remote || f->dlc!=8 ||
        (f->id>>8)!=0x29u || !(f->id&255u) || !pairs ||
        !isfinite(reduction) || reduction<=0) return false;
    int32_t p=signed16(f->data), v=signed16(f->data+2), iq=signed16(f->data+4);
    if (p < -32000 || p > 32000 || v < -32000 || v > 32000 || iq < -6000 || iq > 6000) return false;
    r->id=(uint8_t)f->id; r->position_rad=p*0.1*0.017453292519943295;
    r->velocity_rad_s=v*10.0*0.10471975511965977/(pairs*reduction);
    r->iq_A=iq*0.01;
    r->temperature_C=(int8_t)f->data[6]; r->error=f->data[7];
    return true;
}
bipe_frame bipe_disable(uint8_t id) {
    bipe_frame f={0}; f.id=0xF00u|id; f.extended=true; return f;
}
bool bipe_unwrap(double prev, double x, double last, double dt, double speed,
                 double *total) {
    const double pi=3.14159265358979323846;
    if (!total || !isfinite(prev) || !isfinite(x) || !isfinite(last) ||
        !isfinite(dt) || !isfinite(speed) || dt<=0 || speed<=0 || speed*dt>=pi)
        return false;
    double delta=remainder(x-prev,2*pi);
    if (fabs(delta)>speed*dt+0.001) return false;
    *total=last+delta; return true;
}
