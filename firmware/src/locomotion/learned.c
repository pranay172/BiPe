/* The learned controller's selection switch and runtime; see learned.h. */
#include "learned.h"
#include "generated_config.h"
#include "locomotion_config.h"
#include <math.h>
#include <string.h>

/* The policy and its classical fallback are the M10 serial robot's. */
#if !BIPE_SERIAL
#error "learned.c needs the M10 generated_config.h (make m10-config)"
#endif

static const double NOMINAL[BIPE_LEARNED_POSTURES][4] = BIPE_LEARNED_NOMINAL;
static const double SPEED_RANGE[2] = BIPE_LEARNED_SPEED_RANGE;
static const double YAW_RATE_RANGE[2] = BIPE_LEARNED_YAW_RATE_RANGE;

static bool torque_state(enum bipe_state state) {
    return state == BIPE_ACTIVE || state == BIPE_STOPPING || state == BIPE_FALLING;
}
static bool choosable(enum bipe_state state) {
    return state == BIPE_OFF || state == BIPE_DISCOVERING || state == BIPE_CALIBRATING ||
           state == BIPE_READY;
}
static double clamp(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
static bool finite(const double *v, unsigned n) {
    for (unsigned i = 0; i < n; i++) if (!isfinite(v[i])) return false;
    return true;
}

void bipe_selector_init(bipe_selector *s, const char *checkpoint) {
    memset(s, 0, sizeof *s);
    s->learned_available = checkpoint && strcmp(checkpoint, BIPE_LEARNED_CHECKPOINT_SHA256) == 0;
    s->chosen = s->learned_available ? BIPE_CONTROLLER_DEFAULT : BIPE_CONTROLLER_CLASSICAL;
    s->running = s->chosen;
}
bool bipe_selector_choose(bipe_selector *s, enum bipe_controller c, enum bipe_state state) {
    if (!choosable(state)) return false;
    if (c == BIPE_CONTROLLER_LEARNED && (!s->learned_available || s->locked_out)) return false;
    if (c != BIPE_CONTROLLER_LEARNED && c != BIPE_CONTROLLER_CLASSICAL) return false;
    s->chosen = s->running = c; /* torque is off, so it is taken up at once */
    return true;
}
enum bipe_controller bipe_selector_update(bipe_selector *s, enum bipe_state state) {
    if (!torque_state(state))
        s->running = s->locked_out || !s->learned_available ? BIPE_CONTROLLER_CLASSICAL : s->chosen;
    return s->running;
}
void bipe_selector_handover(bipe_selector *s) {
    s->running = s->chosen = BIPE_CONTROLLER_CLASSICAL;
    s->locked_out = true;
    s->handovers++;
}

void bipe_learned_nominal(double label, double q[4]) {
    double x = (clamp(label, BIPE_LEARNED_LABEL_FIRST,
                      BIPE_LEARNED_LABEL_FIRST + (BIPE_LEARNED_POSTURES - 1) * BIPE_LEARNED_LABEL_STEP) -
                BIPE_LEARNED_LABEL_FIRST) / BIPE_LEARNED_LABEL_STEP;
    int i = (int)floor(x);
    if (i > BIPE_LEARNED_POSTURES - 2) i = BIPE_LEARNED_POSTURES - 2;
    double w = x - i;
    for (unsigned j = 0; j < 4; j++) q[j] = NOMINAL[i][j] * (1 - w) + NOMINAL[i + 1][j] * w;
}
double bipe_learned_label(const double qref[4]) {
    double d[BIPE_LEARNED_POSTURES];
    int i = 0;
    for (int k = 0; k < BIPE_LEARNED_POSTURES; k++) {
        double s = 0;
        for (unsigned j = 0; j < 4; j++) s += (NOMINAL[k][j] - qref[j]) * (NOMINAL[k][j] - qref[j]);
        d[k] = sqrt(s);
        if (d[k] < d[i]) i = k;
    }
    int j = i == 0 || (i < BIPE_LEARNED_POSTURES - 1 && d[i + 1] < d[i - 1]) ? i + 1 : i - 1;
    int a = i < j ? i : j, b = i < j ? j : i;
    double dot = 0, norm = 0;
    for (unsigned k = 0; k < 4; k++) {
        double seg = NOMINAL[b][k] - NOMINAL[a][k];
        dot += (qref[k] - NOMINAL[a][k]) * seg; norm += seg * seg;
    }
    return BIPE_LEARNED_LABEL_FIRST + (a + clamp(dot / norm, 0, 1) * (b - a)) * BIPE_LEARNED_LABEL_STEP;
}
void bipe_learned_init(bipe_learned *l, const double qref[4]) {
    memset(l, 0, sizeof *l);
    l->command[2] = finite(qref, 4) ? bipe_learned_label(qref) : BIPE_LEARNED_LABEL_CENTRE;
    bipe_learned_nominal(l->command[2], l->leg_target);
    memcpy(l->leg_output, l->leg_target, sizeof l->leg_output);
}

enum bipe_learned_status bipe_learned_tick(bipe_learned *l, const bipe_learned_sensors *s,
                                           double velocity, double yaw_rate,
                                           const double qref[4]) {
    if (!s || !qref || !finite(s->gyro, 3) || !finite(s->accel, 3) || !finite(s->q, 4) ||
        !finite(s->qd, 4) || !finite(s->wheel_speed, 2) || !isfinite(velocity) ||
        !isfinite(yaw_rate) || !finite(qref, 4))
        return BIPE_LEARNED_BAD_INPUT;
    if (l->ticks % BIPE_LEARNED_POLICY_TICKS) { l->ticks++; return BIPE_LEARNED_OK; }
    /* The training command: rate limited at the policy step, then observed. */
    const double step = BIPE_LEARNED_POLICY_TICKS * BIPE_LEARNED_TICK_S;
    const double target[3] = {clamp(velocity, SPEED_RANGE[0], SPEED_RANGE[1]),
                              clamp(yaw_rate, YAW_RATE_RANGE[0], YAW_RATE_RANGE[1]),
                              bipe_learned_label(qref)};
    const double rate[3] = {BIPE_LEARNED_SPEED_RATE, BIPE_LEARNED_YAW_RATE_RATE,
                            BIPE_LEARNED_LABEL_RATE};
    double command[3], nominal[4];
    for (unsigned i = 0; i < 3; i++)
        command[i] = rate[i] > 0 ? l->command[i] + clamp(target[i] - l->command[i],
                                                         -rate[i] * step, rate[i] * step)
                                 : target[i];
    bipe_learned_nominal(command[2], nominal);
    float o[BIPE_LOCOMOTION_OBSERVATIONS];
    for (unsigned i = 0; i < 3; i++) { o[i] = (float)s->gyro[i]; o[3 + i] = (float)s->accel[i]; }
    for (unsigned i = 0; i < 4; i++) {
        o[6 + i] = (float)(s->q[i] - nominal[i]);
        o[10 + i] = (float)s->qd[i];
    }
    o[14] = (float)s->wheel_speed[0]; o[15] = (float)s->wheel_speed[1];
    for (unsigned i = 0; i < BIPE_LOCOMOTION_ACTIONS; i++) o[16 + i] = l->action[i];
    o[22] = (float)command[0]; o[23] = (float)command[1];
    o[24] = (float)((command[2] - BIPE_LEARNED_LABEL_CENTRE) / BIPE_LEARNED_LABEL_HALF_SPAN);
    /* A finite double beyond the float range rounds to infinity. */
    for (unsigned i = 0; i < BIPE_LOCOMOTION_OBSERVATIONS; i++)
        if (!isfinite(o[i])) return BIPE_LEARNED_BAD_INPUT;
    memcpy(l->command, command, sizeof command);
    memcpy(l->observation, o, sizeof o);
    l->ticks++;
    l->due = true;
    return BIPE_LEARNED_OK;
}

enum bipe_learned_status bipe_learned_forward(bipe_learned *l, bipe_forward_fn forward) {
    if (!l->due || !forward) return BIPE_LEARNED_OK;
    l->due = false;
    float action[BIPE_LOCOMOTION_ACTIONS];
    forward(l->observation, action);
    for (unsigned i = 0; i < BIPE_LOCOMOTION_ACTIONS; i++)
        if (!isfinite(action[i])) return BIPE_LEARNED_BAD_OUTPUT;
    memcpy(l->action, action, sizeof action);
    double nominal[4];
    bipe_learned_nominal(l->command[2], nominal);
    bool clamped = false;
    for (unsigned j = 0; j < 4; j++) {
        double lo = j % 2 ? BIPE_LEARNED_KNEE_MIN : BIPE_LEARNED_HIP_MIN;
        double hi = j % 2 ? BIPE_LEARNED_KNEE_MAX : BIPE_LEARNED_HIP_MAX;
        double target = nominal[j] + BIPE_LEARNED_LEG_SCALE * action[j];
        l->leg_target[j] = clamp(target, lo, hi);
        clamped = clamped || l->leg_target[j] != target;
    }
    l->leg_clamps += clamped;
    const double forward_speed = l->command[0] / BIPE_LEARNED_WHEEL_RADIUS;
    const double turn = l->command[1] * BIPE_LEARNED_WHEEL_TRACK / (2 * BIPE_LEARNED_WHEEL_RADIUS);
    clamped = false;
    for (unsigned j = 0; j < 2; j++) {
        double target = BIPE_LEARNED_WHEEL_SCALE * action[4 + j] +
                        (j ? forward_speed + turn : forward_speed - turn);
        l->wheel_target[j] = clamp(target, -BIPE_LEARNED_HUB_SPEED_MAX, BIPE_LEARNED_HUB_SPEED_MAX);
        clamped = clamped || l->wheel_target[j] != target;
    }
    l->wheel_clamps += clamped;
    return BIPE_LEARNED_OK;
}

void bipe_learned_legs(bipe_learned *l, double q[4]) {
    const double a = BIPE_LEARNED_TICK_S / (BIPE_LEARNED_LEG_FILTER_TAU_S + BIPE_LEARNED_TICK_S);
    for (unsigned j = 0; j < 4; j++) {
        const double t = l->leg_target[j], band = BIPE_LEARNED_LEG_FILTER_BAND_RAD;
        /* Off (time constant 0): the target exactly, as trained. */
        l->leg_output[j] = BIPE_LEARNED_LEG_FILTER_TAU_S > 0
                               ? clamp(l->leg_output[j] + a * (t - l->leg_output[j]), t - band, t + band)
                               : t;
        q[j] = l->leg_output[j];
    }
}

bool bipe_learned_wheels(bipe_learned *l, const double hub_speed[2], double dt,
                         double torque[2]) {
    if (!hub_speed || !finite(hub_speed, 2) || !isfinite(dt) || dt <= 0) return false;
    const double limit = BIPE_LEARNED_WHEEL_PEAK / BIPE_LEARNED_WHEEL_KI;
    for (unsigned j = 0; j < 2; j++) {
        double error = l->wheel_target[j] - hub_speed[j];
        l->wheel_integral[j] = clamp(l->wheel_integral[j] + error * dt, -limit, limit);
        torque[j] = clamp(BIPE_LEARNED_WHEEL_KV * error + BIPE_LEARNED_WHEEL_KI * l->wheel_integral[j],
                          -BIPE_LEARNED_WHEEL_PEAK, BIPE_LEARNED_WHEEL_PEAK);
    }
    return true;
}

void bipe_drive_init(bipe_drive *d, const char *checkpoint) {
    memset(d, 0, sizeof *d);
    bipe_selector_init(&d->selector, checkpoint);
    d->previous = BIPE_CONTROLLER_CLASSICAL;
}
bool bipe_drive_forward(bipe_drive *d, bipe_forward_fn forward) {
    if (d->selector.running != BIPE_CONTROLLER_LEARNED || !d->learned.due || !forward) return false;
    if (bipe_learned_forward(&d->learned, forward) == BIPE_LEARNED_BAD_OUTPUT)
        bipe_selector_handover(&d->selector);
    return true;
}
bool bipe_drive_step(bipe_drive *d, enum bipe_state state, const bipe_estimate *e,
                     const bipe_learned_sensors *s, const double hub_speed[2],
                     double velocity, double yaw_rate, const double qref[4],
                     const double qdref[4], double dt, bipe_forward_fn forward,
                     bipe_drive_output *out) {
    if (!d || !e || !qref || !qdref || !out) return false;
    memset(out, 0, sizeof *out);
    const bool torque = torque_state(state);
    enum bipe_controller c = bipe_selector_update(&d->selector, state);
    if (c == BIPE_CONTROLLER_LEARNED) {
        /* Fresh at arming, as the policy started in simulation. */
        if (d->previous != c || (torque && !d->torque)) bipe_learned_init(&d->learned, qref);
        d->previous = c; d->torque = torque;
        if (!hub_speed || bipe_learned_tick(&d->learned, s, velocity, yaw_rate, qref) != BIPE_LEARNED_OK)
            return false;
        (void)bipe_drive_forward(d, forward);
        if (d->selector.running == BIPE_CONTROLLER_LEARNED && !torque) {
            /* Torque off: the policy runs (so its time is measured) but its output is
               held at rest, nominal legs and still wheels with no integral, because the
               frames of the tick that arms are computed before the supervisor arms. */
            out->controller = c;
            out->wheel_speed_mode = true;
            bipe_learned_nominal(bipe_learned_label(qref), out->qref);
            return true;
        }
        if (d->selector.running == BIPE_CONTROLLER_LEARNED) {
            out->controller = c;
            bipe_learned_legs(&d->learned, out->qref);
            if (!bipe_learned_wheels(&d->learned, hub_speed, dt, out->wheel_torque)) return false;
            out->wheel_speed_mode = true;
            for (unsigned j = 0; j < 2; j++) {
                out->wheel_speed[j] = d->learned.wheel_target[j];
                out->wheel_torque[j] = BIPE_LEARNED_WHEEL_KI * d->learned.wheel_integral[j];
            }
            return true;
        }
        c = BIPE_CONTROLLER_CLASSICAL; /* handed over in this tick */
    }
    /* Fresh at arming too, as the classical controller starts in simulation. */
    if (d->previous != c || (torque && !d->torque)) memset(&d->balance, 0, sizeof d->balance);
    d->previous = c; d->torque = torque;
    out->controller = c;
    double torques[6], reference;
    if (!bipe_control_step(&d->balance, e, velocity, yaw_rate, qref, qdref, dt, torques, out->ff,
                           &reference))
        return false;
    memcpy(out->qref, qref, sizeof out->qref);
    memcpy(out->qdref, qdref, sizeof out->qdref);
    out->wheel_torque[0] = torques[4]; out->wheel_torque[1] = torques[5];
    return true;
}
