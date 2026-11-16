#include "follower.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* A kinematic robot that follows its own commands exactly, a target that stands and then
   walks, and a Pi that reports every 35 ms with a 130 ms latency. */
typedef struct { double t, range, bearing; uint32_t sequence; bool detected; } pending;
static bipe_follower f;
static bipe_follow_config c;
static double x, y, yaw, v, w, now;
static pending queue[16];
static unsigned queued;
static uint32_t sequence;

static void target(double t, double *tx, double *ty) {
    double walk = t > 1.0 ? 0.5 * (t - 1.0) : 0.0;
    *tx = 1.0 + c.camera_ahead_m + walk; *ty = 0.0;
}
static void step(bool visible) {
    now += c.dt;
    /* A frame every 35 ms: the report leaves after the latency. */
    if(fmod(now + 1e-9, 0.035) < c.dt && queued < 16u) {
        double tx, ty; target(now, &tx, &ty);
        double cx = x + c.camera_ahead_m * cos(yaw), cy = y + c.camera_ahead_m * sin(yaw);
        queue[queued++] = (pending){now + 0.13, hypot(tx - cx, ty - cy),
            remainder(atan2(ty - cy, tx - cx) - yaw, 6.283185307179586), ++sequence, visible};
    }
    while(queued && queue[0].t <= now + 1e-12) {
        bipe_target_report r = {queue[0].sequence, 0, queue[0].detected, queue[0].range,
                                queue[0].bearing, 0.13 - c.link_delay_s};
        if(!r.detected) r.range_m = r.bearing_rad = 0;
        enum bipe_follow_report used = bipe_follow_report_in(&f, &c, &r, now);
        assert(used != BIPE_FOLLOW_INVALID && (used != BIPE_FOLLOW_TOO_OLD || f.count < 30u));
        memmove(queue, queue + 1, (--queued) * sizeof(queue[0]));
    }
    assert(bipe_follow_step(&f, &c, now, x, y, yaw, 0.0, v, true, &v, &w));
    x += v * cos(yaw) * c.dt; y += v * sin(yaw) * c.dt; yaw += w * c.dt;
    assert(fabs(v) <= c.speed_max + 1e-12 && fabs(w) <= c.yaw_rate_max + 1e-12);
}

int main(void) {
    c = bipe_follow_default_config();
    assert(bipe_follow_config_valid(&c));
    bipe_follow_config bad = c; bad.dt = 0; assert(!bipe_follow_config_valid(&bad));
    bad = c; bad.camera_ahead_per_lean = NAN; assert(!bipe_follow_config_valid(&bad));
    bad = c; bad.table_n = BIPE_FOLLOW_TABLE + 1u; assert(!bipe_follow_config_valid(&bad));
    bad = c; bad.history_s = 0.4 * c.dt; bad.max_age_s = bad.history_s;
    assert(!bipe_follow_config_valid(&bad)); /* a history of no samples */
    bipe_follow_init(&f);
    /* Off without a request: reports are ignored and the commands are zero. */
    for(unsigned k = 0; k < 100u; k++) step(true);
    assert(f.mode == BIPE_FOLLOW_OFF && v == 0 && w == 0 && f.reports[BIPE_FOLLOW_IGNORED] > 0);
    /* Malformed, repeated, wrong-tag and stale reports. */
    bipe_target_report r = {sequence, 0, true, NAN, 0, 0.1};
    bipe_follow_request(&f, true);
    assert(bipe_follow_report_in(&f, &c, &r, now) == BIPE_FOLLOW_INVALID);
    r.range_m = 1.0; r.sequence = 1000u; r.tag_id = 3u;
    assert(bipe_follow_report_in(&f, &c, &r, now) == BIPE_FOLLOW_WRONG_ID);
    assert(bipe_follow_report_in(&f, &c, &r, now) == BIPE_FOLLOW_REPEATED);
    r.tag_id = 0; r.sequence++; r.age_s = 2 * c.max_age_s;
    assert(bipe_follow_report_in(&f, &c, &r, now) == BIPE_FOLLOW_TOO_OLD);
    bipe_follow_init(&f); bipe_follow_request(&f, true);
    /* Engage at rest after five detections, then follow the walking target. */
    for(unsigned k = 0; k < 600u; k++) step(true);
    assert(f.mode == BIPE_FOLLOW_FOLLOWING);
    double range, bearing;
    assert(bipe_follow_estimate(&f, &c, &range, &bearing));
    assert(fabs(range - 1.0) < 0.15 && fabs(bearing) < 0.02 && fabs(v - 0.5) < 0.1);
    /* Lost: coast, brake at the limit, hold. */
    bool coasted = false, braked = false;
    for(unsigned k = 0; k < 400u; k++) {
        step(false);
        coasted = coasted || f.mode == BIPE_FOLLOW_COASTING;
        braked = braked || f.mode == BIPE_FOLLOW_BRAKING;
    }
    assert(coasted && braked && f.mode == BIPE_FOLLOW_HOLDING && v == 0 && w == 0);
    /* A late report is still repeated; a Pi that restarts its counter is heard again. */
    uint32_t last = f.last_sequence;
    assert(last > 40u);
    bipe_target_report e = {last - 5u, 0, false, 0, 0, 0.1};
    assert(bipe_follow_report_in(&f, &c, &e, now) == BIPE_FOLLOW_REPEATED);
    e.sequence = 1u;
    assert(bipe_follow_report_in(&f, &c, &e, now) == BIPE_FOLLOW_EMPTY && f.last_sequence == 1u);
    /* A new request accepts the counter afresh. */
    bipe_follow_request(&f, false); bipe_follow_request(&f, true);
    assert(bipe_follow_report_in(&f, &c, &e, now) == BIPE_FOLLOW_EMPTY);
    /* Cancelled: off, zero commands. */
    bipe_follow_request(&f, false);
    step(true);
    assert(f.mode == BIPE_FOLLOW_OFF && v == 0 && w == 0);
    /* Not allowed (supervisor not active): the request is dropped. */
    bipe_follow_request(&f, true);
    assert(bipe_follow_step(&f, &c, now + c.dt, x, y, yaw, 0, 0, false, &v, &w));
    assert(f.mode == BIPE_FOLLOW_OFF && f.event == BIPE_FOLLOW_EVENT_CANCELLED);
    /* A non-finite lean is rejected. */
    assert(!bipe_follow_step(&f, &c, now + 2 * c.dt, x, y, yaw, NAN, 0, true, &v, &w));
    /* Time must advance. */
    assert(!bipe_follow_step(&f, &c, now, x, y, yaw, 0, 0, true, &v, &w) && v == 0);
    puts("follower: config checks, off, malformed/wrong-tag/stale reports, engage, follow, coast, brake, hold, cancel, not allowed, time order pass; sequence restart and re-request pass");
    return 0;
}
