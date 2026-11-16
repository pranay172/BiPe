#include "follower.h"
#include "follower_config.h"
#include <math.h>
#include <string.h>

/* The Python reference (bipe/m12/follow.py) mirrors this file operation by operation;
   change both together and run the parity tests. */

static const double TWO_PI = 6.28318530717958647692;
/* A sequence this far behind the last one is the Pi restarting its counter (one second of
   frames at 30 Hz), not a late or repeated report. */
static const uint32_t SEQUENCE_RESTART_GAP = 30u;

bipe_follow_config bipe_follow_default_config(void) {
    bipe_follow_config c = BIPE_FOLLOW_DEFAULT_CONFIG;
    return c;
}

static bool positive(double v) { return isfinite(v) && v > 0; }

bool bipe_follow_config_valid(const bipe_follow_config *c) {
    if(!c || !positive(c->dt) || !positive(c->history_s) || !positive(c->max_age_s) ||
       !isfinite(c->link_delay_s) || c->link_delay_s < 0 || !positive(c->frame_s) ||
       !isfinite(c->camera_ahead_m) || !isfinite(c->camera_ahead_per_lean) || !positive(c->process_density) ||
       !positive(c->initial_velocity_sigma) || c->table_n < 2u || c->table_n > BIPE_FOLLOW_TABLE ||
       !positive(c->noise_scale) || !positive(c->range_std_floor) ||
       !positive(c->bearing_std_floor) || !positive(c->innovation_chi2) || !positive(c->gate_m) ||
       c->engage_detections < 1u || !isfinite(c->engage_s) || c->engage_s < 0 ||
       !positive(c->engage_range_min) || !(c->engage_range_max > c->engage_range_min) ||
       !positive(c->engage_speed_max) || !positive(c->coast_after_frames) ||
       !positive(c->coast_s) || !positive(c->stale_s) || !positive(c->hold_speed) ||
       !positive(c->nominal_range) || !positive(c->range_gain) || !positive(c->bearing_gain) ||
       !isfinite(c->stop_range) || c->stop_range < 0 || !positive(c->lateral_range_floor) ||
       !positive(c->speed_max) || !isfinite(c->reverse_max) || c->reverse_max < 0 ||
       !positive(c->acceleration_max) || !positive(c->yaw_rate_max) ||
       !positive(c->yaw_acceleration_max) || !positive(c->brake_deceleration))
        return false;
    double needed = c->history_s / c->dt;
    if(needed > (double)BIPE_FOLLOW_HISTORY || lround(needed) < 2 || c->max_age_s > c->history_s)
        return false;
    for(unsigned i = 0; i < c->table_n; i++) {
        if(!positive(c->table_range[i]) || !positive(c->table_range_std[i]) ||
           !positive(c->table_bearing_std[i])) return false;
        if(i > 0 && !(c->table_range[i] > c->table_range[i - 1])) return false;
    }
    return true;
}

void bipe_follow_init(bipe_follower *f) {
    memset(f, 0, sizeof(*f));
    f->mode = BIPE_FOLLOW_OFF;
}

void bipe_follow_request(bipe_follower *f, bool on) {
    if(on) {
        if(f->mode == BIPE_FOLLOW_OFF) {
            f->mode = BIPE_FOLLOW_WAITING;
            f->streak = 0;
            f->sequence_seen = false; /* a new request accepts the Pi's counter afresh */
        }
        f->requested = true;
    } else {
        if(f->mode != BIPE_FOLLOW_OFF) f->pending = BIPE_FOLLOW_EVENT_CANCELLED;
        f->requested = false;
        f->mode = BIPE_FOLLOW_OFF;
        f->streak = 0;
        f->velocity = 0;
        f->yaw_rate = 0;
    }
}

static double wrap(double a) { return remainder(a, TWO_PI); }

static void predict(double s[4], double P[16], double dt, double q) {
    double A[16], B[16];
    s[0] += dt * s[2];
    s[1] += dt * s[3];
    for(unsigned i = 0; i < 4; i++)
        for(unsigned j = 0; j < 4; j++)
            A[4 * i + j] = P[4 * i + j] + (i < 2 ? dt * P[4 * (i + 2) + j] : 0.0);
    for(unsigned i = 0; i < 4; i++)
        for(unsigned j = 0; j < 4; j++)
            B[4 * i + j] = A[4 * i + j] + (j < 2 ? dt * A[4 * i + j + 2] : 0.0);
    double q3 = q * dt * dt * dt / 3.0, q2 = q * dt * dt / 2.0, q1 = q * dt;
    B[0] += q3; B[5] += q3;
    B[2] += q2; B[8] += q2; B[7] += q2; B[13] += q2;
    B[10] += q1; B[15] += q1;
    memcpy(P, B, sizeof(B));
}

static void noise(const bipe_follow_config *c, double r, double *range_std, double *bearing_std) {
    unsigned n = c->table_n;
    double rs, bs;
    if(r <= c->table_range[0]) {
        rs = c->table_range_std[0]; bs = c->table_bearing_std[0];
    } else if(r >= c->table_range[n - 1]) {
        rs = c->table_range_std[n - 1]; bs = c->table_bearing_std[n - 1];
    } else {
        unsigned i = 0;
        while(r > c->table_range[i + 1]) i++;
        double w = (r - c->table_range[i]) / (c->table_range[i + 1] - c->table_range[i]);
        rs = c->table_range_std[i] + w * (c->table_range_std[i + 1] - c->table_range_std[i]);
        bs = c->table_bearing_std[i] + w * (c->table_bearing_std[i + 1] - c->table_bearing_std[i]);
    }
    rs *= c->noise_scale;
    bs *= c->noise_scale;
    *range_std = rs > c->range_std_floor ? rs : c->range_std_floor;
    *bearing_std = bs > c->bearing_std_floor ? bs : c->bearing_std_floor;
}

/* The report as a point in the odometry frame and its covariance, from the camera at the
   pose (x, y, yaw), ahead of the axle by ahead. */
static void measurement(const bipe_follow_config *c, double x, double y, double yaw,
                        double ahead, double r, double b, double z[2], double R[4]) {
    double camx = x + ahead * cos(yaw), camy = y + ahead * sin(yaw);
    double th = yaw + b, ct = cos(th), st = sin(th);
    z[0] = camx + r * ct;
    z[1] = camy + r * st;
    double rs, bs;
    noise(c, r, &rs, &bs);
    double vr = rs * rs, vt = (r * bs) * (r * bs);
    R[0] = vr * ct * ct + vt * st * st;
    R[1] = vr * ct * st - vt * ct * st;
    R[2] = R[1];
    R[3] = vr * st * st + vt * ct * ct;
}

/* Squared Mahalanobis distance and Euclidean length of the innovation. */
static void innovation(const double s[4], const double P[16], const double z[2],
                       const double R[4], double *d2, double *distance) {
    double y0 = z[0] - s[0], y1 = z[1] - s[1];
    double S0 = P[0] + R[0], S1 = P[1] + R[1], S2 = P[4] + R[2], S3 = P[5] + R[3];
    double det = S0 * S3 - S1 * S2;
    *d2 = (y0 * (S3 * y0 - S1 * y1) + y1 * (S0 * y1 - S2 * y0)) / det;
    *distance = sqrt(y0 * y0 + y1 * y1);
}

static void update(double s[4], double P[16], const double z[2], const double R[4]) {
    double y0 = z[0] - s[0], y1 = z[1] - s[1];
    double S0 = P[0] + R[0], S1 = P[1] + R[1], S2 = P[4] + R[2], S3 = P[5] + R[3];
    double det = S0 * S3 - S1 * S2;
    double I0 = S3 / det, I1 = -S1 / det, I2 = -S2 / det, I3 = S0 / det;
    double K[8], N[16];
    for(unsigned i = 0; i < 4; i++) {
        K[2 * i] = P[4 * i] * I0 + P[4 * i + 1] * I2;
        K[2 * i + 1] = P[4 * i] * I1 + P[4 * i + 1] * I3;
    }
    for(unsigned i = 0; i < 4; i++) {
        s[i] += K[2 * i] * y0 + K[2 * i + 1] * y1;
        for(unsigned j = 0; j < 4; j++)
            N[4 * i + j] = P[4 * i + j] - (K[2 * i] * P[j] + K[2 * i + 1] * P[4 + j]);
    }
    for(unsigned i = 0; i < 4; i++)
        for(unsigned j = 0; j < 4; j++)
            P[4 * i + j] = 0.5 * (N[4 * i + j] + N[4 * j + i]);
}

static unsigned history_length(const bipe_follow_config *c) {
    unsigned n = (unsigned)lround(c->history_s / c->dt);
    return n < BIPE_FOLLOW_HISTORY ? n : BIPE_FOLLOW_HISTORY;
}

/* Index of the sample nearest to time t, or BIPE_FOLLOW_HISTORY if t is outside it. */
static unsigned nearest(const bipe_follower *f, const bipe_follow_config *c, double t) {
    if(f->count == 0u) return BIPE_FOLLOW_HISTORY;
    unsigned n = history_length(c);
    unsigned oldest = (f->head + n + 1u - f->count) % n;
    if(t < f->history[oldest].t - 0.5 * c->dt) return BIPE_FOLLOW_HISTORY;
    unsigned best = f->head;
    double gap = fabs(f->history[f->head].t - t);
    for(unsigned k = 1u; k < f->count; k++) {
        unsigned i = (f->head + n - k) % n;
        double g = fabs(f->history[i].t - t);
        if(g < gap) { gap = g; best = i; }
        if(f->history[i].t < t) break;
    }
    return best;
}

/* After sample k changed, predict it forward through the later samples to the latest. */
static void repropagate(bipe_follower *f, const bipe_follow_config *c, unsigned k) {
    unsigned n = history_length(c);
    double s[4], P[16];
    memcpy(s, f->history[k].s, sizeof(s));
    memcpy(P, f->history[k].P, sizeof(P));
    double t = f->history[k].t;
    unsigned i = k;
    while(i != f->head) {
        i = (i + 1u) % n;
        predict(s, P, f->history[i].t - t, c->process_density);
        t = f->history[i].t;
        memcpy(f->history[i].s, s, sizeof(s));
        memcpy(f->history[i].P, P, sizeof(P));
    }
    memcpy(f->s, s, sizeof(s));
    memcpy(f->P, P, sizeof(P));
}

static void initialize(bipe_follower *f, const bipe_follow_config *c, unsigned k,
                       const double z[2], const double R[4]) {
    bipe_follow_sample *h = &f->history[k];
    double v = c->initial_velocity_sigma * c->initial_velocity_sigma;
    memset(h->s, 0, sizeof(h->s));
    memset(h->P, 0, sizeof(h->P));
    h->s[0] = z[0];
    h->s[1] = z[1];
    h->P[0] = R[0]; h->P[1] = R[1]; h->P[4] = R[2]; h->P[5] = R[3];
    h->P[10] = v; h->P[15] = v;
    f->initialized = true;
    repropagate(f, c, k);
}

enum bipe_follow_report bipe_follow_report_in(bipe_follower *f, const bipe_follow_config *c,
    const bipe_target_report *r, double now) {
    if(!isfinite(now) || !isfinite(r->age_s) || r->age_s < 0 ||
       (r->detected && (!isfinite(r->range_m) || !(r->range_m > 0) || !isfinite(r->bearing_rad)))) {
        f->reports[BIPE_FOLLOW_INVALID]++;
        return BIPE_FOLLOW_INVALID;
    }
    if(f->sequence_seen && r->sequence <= f->last_sequence &&
       f->last_sequence - r->sequence <= SEQUENCE_RESTART_GAP) {
        f->reports[BIPE_FOLLOW_REPEATED]++;
        return BIPE_FOLLOW_REPEATED;
    }
    f->sequence_seen = true;
    f->last_sequence = r->sequence;
    f->message_seen = true;
    f->last_message = now;
    enum bipe_follow_report out;
    if(f->mode == BIPE_FOLLOW_OFF) {
        out = BIPE_FOLLOW_IGNORED;
    } else if(!r->detected) {
        f->streak = 0;
        out = BIPE_FOLLOW_EMPTY;
    } else if(r->tag_id != c->tag_id) {
        f->streak = 0;
        out = BIPE_FOLLOW_WRONG_ID;
    } else {
        double t = now - r->age_s - c->link_delay_s;
        unsigned k = r->age_s + c->link_delay_s > c->max_age_s ? BIPE_FOLLOW_HISTORY : nearest(f, c, t);
        if(k == BIPE_FOLLOW_HISTORY) {
            out = BIPE_FOLLOW_TOO_OLD;
        } else {
            bipe_follow_sample *h = &f->history[k];
            double z[2], R[4], d2 = 0, distance = 0;
            measurement(c, h->x, h->y, h->yaw, h->ahead, r->range_m, r->bearing_rad, z, R);
            if(f->initialized) innovation(h->s, h->P, z, R, &d2, &distance);
            if(f->mode == BIPE_FOLLOW_WAITING || f->mode == BIPE_FOLLOW_HOLDING) {
                bool continues = f->streak > 0u && f->initialized &&
                    r->sequence == f->streak_sequence + 1u && distance <= c->gate_m;
                if(continues) {
                    update(h->s, h->P, z, R);
                    repropagate(f, c, k);
                    f->streak++;
                } else {
                    initialize(f, c, k, z, R);
                    f->streak = 1u;
                    f->streak_start = h->t;
                }
                f->streak_sequence = r->sequence;
                f->last_range = r->range_m;
                f->last_fused = h->t;
                f->last_accepted = now;
                out = BIPE_FOLLOW_ACCEPTED;
            } else {
                bool following = f->mode == BIPE_FOLLOW_FOLLOWING;
                bool inside = distance <= c->gate_m || (following && d2 <= c->innovation_chi2);
                if(inside) {
                    update(h->s, h->P, z, R);
                    repropagate(f, c, k);
                    f->last_range = r->range_m;
                    f->last_fused = h->t;
                    f->last_accepted = now;
                    if(!following) {
                        f->mode = BIPE_FOLLOW_FOLLOWING;
                        f->pending = BIPE_FOLLOW_EVENT_REACQUIRED;
                    }
                    out = BIPE_FOLLOW_ACCEPTED;
                } else {
                    out = BIPE_FOLLOW_GATED;
                }
            }
        }
    }
    f->reports[out]++;
    return out;
}

static double toward_zero(double v, double step) {
    if(fabs(v) <= step) return 0.0;
    return v > 0 ? v - step : v + step;
}

static double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void geometry(const bipe_follower *f, double x, double y, double yaw, double ahead,
                     double *r, double u[2], double *bearing) {
    double camx = x + ahead * cos(yaw), camy = y + ahead * sin(yaw);
    double rx = f->s[0] - camx, ry = f->s[1] - camy;
    *r = sqrt(rx * rx + ry * ry);
    double d = *r > 1e-9 ? *r : 1e-9;
    u[0] = rx / d;
    u[1] = ry / d;
    *bearing = wrap(atan2(ry, rx) - yaw);
}

bool bipe_follow_estimate(const bipe_follower *f, const bipe_follow_config *c,
                          double *range, double *bearing) {
    if(!f->initialized || f->count == 0u) return false;
    const bipe_follow_sample *h = &f->history[f->head];
    double u[2];
    (void)c;
    geometry(f, h->x, h->y, h->yaw, h->ahead, range, u, bearing);
    return true;
}

bool bipe_follow_step(bipe_follower *f, const bipe_follow_config *c, double now, double x,
    double y, double yaw, double lean, double robot_velocity, bool allowed, double *velocity,
    double *yaw_rate) {
    if(!isfinite(now) || !isfinite(x) || !isfinite(y) || !isfinite(yaw) || !isfinite(lean) ||
       !isfinite(robot_velocity) || (f->ticked && !(now > f->now))) {
        *velocity = 0;
        *yaw_rate = 0;
        return false;
    }
    double dt = f->ticked ? now - f->now : c->dt;
    f->event = f->pending;
    f->pending = BIPE_FOLLOW_EVENT_NONE;
    if(!allowed && f->mode != BIPE_FOLLOW_OFF) {
        bipe_follow_request(f, false);
        f->event = BIPE_FOLLOW_EVENT_CANCELLED;
        f->pending = BIPE_FOLLOW_EVENT_NONE;
    }
    if(!allowed) f->requested = false;
    /* The filter and the pose history advance to this tick. */
    unsigned n = history_length(c);
    if(f->initialized && f->ticked) predict(f->s, f->P, dt, c->process_density);
    f->head = f->count == 0u ? 0u : (f->head + 1u) % n;
    if(f->count < n) f->count++;
    bipe_follow_sample *h = &f->history[f->head];
    h->t = now; h->x = x; h->y = y; h->yaw = yaw;
    h->ahead = c->camera_ahead_m + c->camera_ahead_per_lean * lean;
    memcpy(h->s, f->s, sizeof(h->s));
    memcpy(h->P, f->P, sizeof(h->P));
    f->now = now;
    f->ticked = true;
    double silent = now - f->last_accepted;
    switch(f->mode) {
    case BIPE_FOLLOW_WAITING:
    case BIPE_FOLLOW_HOLDING: {
        if(f->streak > 0u && silent > c->coast_after_frames * c->frame_s) f->streak = 0;
        if(f->streak >= c->engage_detections &&
           f->last_fused - f->streak_start >= c->engage_s - 1e-9 &&
           f->last_range >= c->engage_range_min && f->last_range <= c->engage_range_max &&
           fabs(robot_velocity) < c->engage_speed_max) {
            f->mode = BIPE_FOLLOW_FOLLOWING;
            f->event = BIPE_FOLLOW_EVENT_ENGAGED;
        }
        break;
    }
    case BIPE_FOLLOW_FOLLOWING:
        if(silent > c->coast_after_frames * c->frame_s) {
            f->mode = BIPE_FOLLOW_COASTING;
            f->event = BIPE_FOLLOW_EVENT_COASTING;
        }
        break;
    default:
        break;
    }
    if((f->mode == BIPE_FOLLOW_COASTING && silent > c->coast_s + 1e-9) ||
       ((f->mode == BIPE_FOLLOW_FOLLOWING || f->mode == BIPE_FOLLOW_COASTING) &&
        now - f->last_message > c->stale_s + 1e-9)) {
        f->mode = BIPE_FOLLOW_BRAKING;
        f->event = BIPE_FOLLOW_EVENT_LOST;
        f->lost_x = x;
        f->lost_y = y;
    }
    double v = 0, w = 0;
    if(f->mode == BIPE_FOLLOW_FOLLOWING || f->mode == BIPE_FOLLOW_COASTING) {
        double r, u[2], bearing;
        /* The law holds the range from the upright camera, a fixed point on the robot. */
        geometry(f, x, y, yaw, c->camera_ahead_m, &r, u, &bearing);
        double along = f->s[2] * u[0] + f->s[3] * u[1];
        double across = -f->s[2] * u[1] + f->s[3] * u[0];
        v = along + c->range_gain * (r - c->nominal_range);
        if(r < c->stop_range) v = 0;
        w = c->bearing_gain * bearing + across / (r > c->lateral_range_floor ? r : c->lateral_range_floor);
        v = clamp(v, -c->reverse_max, c->speed_max);
        w = clamp(w, -c->yaw_rate_max, c->yaw_rate_max);
        v = f->velocity + clamp(v - f->velocity, -c->acceleration_max * dt, c->acceleration_max * dt);
        w = f->yaw_rate + clamp(w - f->yaw_rate, -c->yaw_acceleration_max * dt,
                                c->yaw_acceleration_max * dt);
    } else if(f->mode == BIPE_FOLLOW_BRAKING) {
        v = toward_zero(f->velocity, c->brake_deceleration * dt);
        w = toward_zero(f->yaw_rate, c->yaw_acceleration_max * dt);
        if(v == 0 && w == 0 && fabs(robot_velocity) < c->hold_speed) {
            f->mode = BIPE_FOLLOW_HOLDING;
            f->event = BIPE_FOLLOW_EVENT_HOLDING;
            f->streak = 0;
        }
    }
    f->velocity = v;
    f->yaw_rate = w;
    *velocity = v;
    *yaw_rate = w;
    return true;
}
