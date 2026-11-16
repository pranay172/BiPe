#include "learned.h"
#include "generated_config.h"
#include "locomotion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static float fake_value = 0;
static unsigned fake_calls = 0;
static void fake_forward(const float observation[BIPE_LOCOMOTION_OBSERVATIONS],
                         float action[BIPE_LOCOMOTION_ACTIONS]) {
    (void)observation;
    fake_calls++;
    for (unsigned i = 0; i < BIPE_LOCOMOTION_ACTIONS; i++)
        action[i] = i % 2 ? -fake_value : fake_value;
}
static bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }
static bipe_learned_sensors rest(const double qref[4]) {
    bipe_learned_sensors s = {{0.01, -0.02, 0.03}, {0.1, 0.2, 9.8}, {0}, {0.4, 0.5, 0.6, 0.7}, {1.5, -2.5}};
    memcpy(s.q, qref, sizeof s.q);
    return s;
}

int main(void) {
    const char *ok = BIPE_LEARNED_CHECKPOINT_SHA256;
    /* Selection switch: classical by default; learned only with the matching forward
       pass, only chosen while torque is off, held for the armed run. */
    bipe_selector sel;
    bipe_selector_init(&sel, NULL);
    assert(!sel.learned_available && sel.chosen == BIPE_CONTROLLER_CLASSICAL);
    assert(!bipe_selector_choose(&sel, BIPE_CONTROLLER_LEARNED, BIPE_READY));
    bipe_selector_init(&sel, "0000");
    assert(!bipe_selector_choose(&sel, BIPE_CONTROLLER_LEARNED, BIPE_READY));
    bipe_selector_init(&sel, ok);
    assert(sel.learned_available && sel.chosen == BIPE_CONTROLLER_CLASSICAL);
    assert(bipe_selector_update(&sel, BIPE_OFF) == BIPE_CONTROLLER_CLASSICAL);
    const enum bipe_state off[] = {BIPE_OFF, BIPE_DISCOVERING, BIPE_CALIBRATING, BIPE_READY};
    const enum bipe_state on[] = {BIPE_ACTIVE, BIPE_STOPPING, BIPE_FALLING, BIPE_FAULT, BIPE_FALLEN};
    for (unsigned i = 0; i < 5; i++)
        assert(!bipe_selector_choose(&sel, BIPE_CONTROLLER_LEARNED, on[i]));
    for (unsigned i = 0; i < 4; i++)
        assert(bipe_selector_choose(&sel, BIPE_CONTROLLER_LEARNED, off[i]));
    assert(!bipe_selector_choose(&sel, (enum bipe_controller)7, BIPE_READY));
    assert(bipe_selector_update(&sel, BIPE_READY) == BIPE_CONTROLLER_LEARNED);
    assert(bipe_selector_update(&sel, BIPE_ACTIVE) == BIPE_CONTROLLER_LEARNED);
    assert(!bipe_selector_choose(&sel, BIPE_CONTROLLER_CLASSICAL, BIPE_ACTIVE));
    sel.chosen = BIPE_CONTROLLER_CLASSICAL; /* even a changed choice waits for torque off */
    assert(bipe_selector_update(&sel, BIPE_STOPPING) == BIPE_CONTROLLER_LEARNED);
    assert(bipe_selector_update(&sel, BIPE_READY) == BIPE_CONTROLLER_CLASSICAL);
    assert(bipe_selector_choose(&sel, BIPE_CONTROLLER_LEARNED, BIPE_READY));
    bipe_selector_handover(&sel);
    assert(sel.locked_out && sel.handovers == 1 && sel.running == BIPE_CONTROLLER_CLASSICAL);
    assert(!bipe_selector_choose(&sel, BIPE_CONTROLLER_LEARNED, BIPE_READY));
    assert(bipe_selector_update(&sel, BIPE_READY) == BIPE_CONTROLLER_CLASSICAL);

    /* Posture table: the label inverts the nominal references and is clamped. */
    double q[4], top[4];
    /* As bipe.m13.policy_control: exact at labels and midpoints; between, the neighbour
       pick by distance can choose the far segment and snap to the table label. */
    for (double label = 30; label <= 65; label += 0.5) {
        bipe_learned_nominal(label, q);
        assert(near(bipe_learned_label(q), label, 1e-9));
    }
    for (double label = 30; label <= 65; label += 0.37) {
        bipe_learned_nominal(label, q);
        double found = bipe_learned_label(q);
        assert(near(found, label, 1e-9) || (found == round(label) && near(found, label, 0.5)));
    }
    bipe_learned_nominal(80, q); bipe_learned_nominal(65, top);
    assert(memcmp(q, top, sizeof q) == 0 && bipe_learned_label(q) == 65);
    bipe_learned_nominal(10, q);
    assert(bipe_learned_label(q) == 30);
    for (unsigned k = 0; k < BIPE_LEARNED_POSTURES; k++) {
        bipe_learned_nominal(BIPE_LEARNED_LABEL_FIRST + k * BIPE_LEARNED_LABEL_STEP, q);
        assert(q[0] > BIPE_LEARNED_HIP_MIN && q[0] < BIPE_LEARNED_HIP_MAX);
        assert(q[1] > BIPE_LEARNED_KNEE_MIN && q[1] < BIPE_LEARNED_KNEE_MAX);
    }

    /* Runtime: a policy step every fourth tick, rate-limited command, observation layout. */
    double qref[4];
    bipe_learned_nominal(40, qref);
    bipe_learned l;
    bipe_learned_init(&l, qref);
    assert(near(l.command[2], 40, 1e-9) && memcmp(l.leg_target, qref, sizeof qref) == 0);
    bipe_learned_sensors s = rest(qref);
    s.q[1] += 0.1;
    double target45[4];
    bipe_learned_nominal(45, target45);
    fake_value = 0.25f; fake_calls = 0;
    for (unsigned t = 0; t < 8; t++) {
        assert(bipe_learned_tick(&l, &s, 3.0, -0.4, target45) == BIPE_LEARNED_OK);
        assert(l.due == (t % 4 == 0));
        assert(bipe_learned_forward(&l, fake_forward) == BIPE_LEARNED_OK && !l.due);
    }
    assert(fake_calls == 2 && l.ticks == 8);
    assert(near(l.command[0], 2 * 1.25 * 0.02, 1e-12));       /* speed rate limit */
    assert(l.command[1] == -0.4);                              /* yaw rate unlimited */
    assert(near(l.command[2], 40 + 2 * 20 * 0.02, 1e-9));      /* posture label rate */
    const float *o = l.observation;
    double nominal[4];
    bipe_learned_nominal(l.command[2], nominal);
    for (unsigned i = 0; i < 3; i++) assert(o[i] == (float)s.gyro[i] && o[3 + i] == (float)s.accel[i]);
    for (unsigned i = 0; i < 4; i++)
        assert(o[6 + i] == (float)(s.q[i] - nominal[i]) && o[10 + i] == (float)s.qd[i]);
    assert(o[14] == 1.5f && o[15] == -2.5f);
    assert(o[16] == 0.25f && o[17] == -0.25f && o[21] == -0.25f); /* the previous action */
    assert(o[22] == (float)l.command[0] && o[23] == -0.4f);
    assert(o[24] == (float)((l.command[2] - 47.5) / 17.5));
    for (unsigned i = 0; i < 300; i++) {
        assert(bipe_learned_tick(&l, &s, 3.0, -4.0, target45) == BIPE_LEARNED_OK);
        bipe_learned_forward(&l, fake_forward);
    }
    assert(l.command[0] == 1.2 && l.command[1] == -1.0);       /* trained ranges */
    assert(near(l.command[2], 45, 1e-9));
    /* Action law. */
    for (unsigned j = 0; j < 4; j++) assert(near(l.leg_target[j], target45[j] + 0.5 * (j % 2 ? -0.25 : 0.25), 1e-12));
    const double v = 1.2 / 0.036, w = -1.0 * 0.2 / (2 * 0.036);
    assert(near(l.wheel_target[0], 40 * 0.25 + v - w, 1e-9) && near(l.wheel_target[1], -40 * 0.25 + v + w, 1e-9));
    assert(l.leg_clamps == 0 && l.wheel_clamps == 0);
    /* Leg target filter: first order from leg_output to leg_target, time constant
       BIPE_LEARNED_LEG_FILTER_TAU_S at the control tick, kept within
       BIPE_LEARNED_LEG_FILTER_BAND_RAD of the target; init starts it at the target. */
    {
        bipe_learned f;
        bipe_learned_init(&f, qref);
        assert(memcmp(f.leg_output, f.leg_target, sizeof f.leg_output) == 0);
        const double fa = BIPE_LEARNED_TICK_S / (BIPE_LEARNED_LEG_FILTER_TAU_S + BIPE_LEARNED_TICK_S);
        const double band = BIPE_LEARNED_LEG_FILTER_BAND_RAD;
        assert(BIPE_LEARNED_LEG_FILTER_TAU_S >= 0 && band > 0); /* tau 0: off */
        double legs[4], gap = 0.5 * band;
        for (unsigned j = 0; j < 4; j++) f.leg_target[j] += gap;
        for (unsigned t = 0; t < 200; t++) {
            bipe_learned_legs(&f, legs);
            gap *= 1 - fa;
            for (unsigned j = 0; j < 4; j++)
                assert(legs[j] == f.leg_output[j] && near(f.leg_target[j] - legs[j], gap, 1e-12));
        }
        assert(gap < 1e-9);
        /* A large move passes at once to within the band, then filters. */
        for (unsigned j = 0; j < 4; j++) f.leg_target[j] -= 10 * band;
        double d = fmin(band, 10 * band * (1 - fa));
        bipe_learned_legs(&f, legs);
        for (unsigned j = 0; j < 4; j++) assert(near(legs[j] - f.leg_target[j], d, 1e-12));
        d = fmin(band, d * (1 - fa));
        bipe_learned_legs(&f, legs);
        for (unsigned j = 0; j < 4; j++) assert(near(legs[j] - f.leg_target[j], d, 1e-12));
    }

    /* Output guards: an unbounded action is clamped to the joint limits and hub speed. */
    bipe_learned big = l;
    fake_value = 1e6f;
    while (!big.due) assert(bipe_learned_tick(&big, &s, 0, 0, target45) == BIPE_LEARNED_OK);
    assert(bipe_learned_forward(&big, fake_forward) == BIPE_LEARNED_OK);
    assert(big.leg_target[0] == BIPE_LEARNED_HIP_MAX && big.leg_target[1] == BIPE_LEARNED_KNEE_MIN);
    assert(big.leg_target[2] == BIPE_LEARNED_HIP_MAX && big.leg_target[3] == BIPE_LEARNED_KNEE_MIN);
    assert(big.wheel_target[0] == BIPE_LEARNED_HUB_SPEED_MAX && big.wheel_target[1] == -BIPE_LEARNED_HUB_SPEED_MAX);
    assert(big.leg_clamps == 1 && big.wheel_clamps == 1);
    double torque[2], hub[2] = {0, 0};
    for (unsigned i = 0; i < 2000; i++) assert(bipe_learned_wheels(&big, hub, 0.005, torque));
    assert(torque[0] == BIPE_LEARNED_WHEEL_PEAK && torque[1] == -BIPE_LEARNED_WHEEL_PEAK);
    assert(near(big.wheel_integral[0], BIPE_LEARNED_WHEEL_PEAK / BIPE_LEARNED_WHEEL_KI, 1e-12));
    hub[0] = NAN; assert(!bipe_learned_wheels(&big, hub, 0.005, torque)); hub[0] = 0;
    assert(!bipe_learned_wheels(&big, hub, 0, torque));
    /* Wheel speed loop law below the clamps. */
    bipe_learned pi = l;
    memset(pi.wheel_integral, 0, sizeof pi.wheel_integral);
    hub[0] = pi.wheel_target[0] - 0.4; hub[1] = pi.wheel_target[1] + 0.2;
    assert(bipe_learned_wheels(&pi, hub, 0.005, torque));
    assert(near(torque[0], 0.05 * 0.4 + 0.5 * 0.4 * 0.005, 1e-12));
    assert(near(torque[1], -0.05 * 0.2 - 0.5 * 0.2 * 0.005, 1e-12));

    /* Non-finite output (NaN, infinity): nothing reaches the targets or the held action. */
    const float bad[2] = {NAN, INFINITY};
    for (unsigned k = 0; k < 2; k++) {
        bipe_learned n = l;
        fake_value = bad[k];
        while (!n.due) assert(bipe_learned_tick(&n, &s, 0, 0, target45) == BIPE_LEARNED_OK);
        bipe_learned before = n;
        assert(bipe_learned_forward(&n, fake_forward) == BIPE_LEARNED_BAD_OUTPUT && !n.due);
        assert(memcmp(n.action, before.action, sizeof n.action) == 0);
        assert(memcmp(n.leg_target, before.leg_target, sizeof n.leg_target) == 0);
        assert(memcmp(n.wheel_target, before.wheel_target, sizeof n.wheel_target) == 0);
    }
    /* Non-finite input, or beyond float range: BAD_INPUT and no change. */
    double *fields[5] = {&s.gyro[1], &s.accel[2], &s.q[3], &s.qd[0], &s.wheel_speed[1]};
    for (unsigned k = 0; k < 5; k++) {
        const double values[2] = {NAN, 1e300};
        for (unsigned m = 0; m < 2; m++) {
            bipe_learned n = l;
            while (n.ticks % 4) assert(bipe_learned_tick(&n, &s, 0, 0, target45) == BIPE_LEARNED_OK);
            bipe_learned before = n;
            double kept = *fields[k];
            *fields[k] = values[m];
            assert(bipe_learned_tick(&n, &s, 0, 0, target45) == BIPE_LEARNED_BAD_INPUT);
            assert(memcmp(&n, &before, sizeof n) == 0);
            *fields[k] = kept;
        }
    }
    assert(bipe_learned_tick(&l, &s, NAN, 0, target45) == BIPE_LEARNED_BAD_INPUT);
    assert(bipe_learned_tick(&l, NULL, 0, 0, target45) == BIPE_LEARNED_BAD_INPUT);

    /* Drive step: classical by default and equal to bipe_control_step. */
    bipe_estimate e = {0};
    double posture[4], qd[4] = {0};
    assert(bipe_posture(40.0, posture)); /* the M10 kinematics: posture label 40 */
    assert(near(bipe_learned_label(posture), 40, 1e-9));
    memcpy(e.q, posture, sizeof posture);
    e.pitch = 0.01; e.pitch_rate = -0.02;
    s = rest(posture);
    hub[0] = hub[1] = 0;
    bipe_drive d;
    bipe_drive_init(&d, ok);
    bipe_balance reference_balance = {0};
    bipe_drive_output out;
    double t6[6], ff[4], pitch;
    for (unsigned t = 0; t < 10; t++) {
        assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0.2, 0, posture, qd, 0.005, fake_forward, &out));
        assert(bipe_control_step(&reference_balance, &e, 0.2, 0, posture, qd, 0.005, t6, ff, &pitch));
        assert(out.controller == BIPE_CONTROLLER_CLASSICAL && !out.wheel_speed_mode);
        assert(out.wheel_torque[0] == t6[4] && out.wheel_torque[1] == t6[5]);
        assert(memcmp(out.ff, ff, sizeof ff) == 0 && memcmp(out.qref, posture, sizeof posture) == 0);
    }
    /* The classical knee feedforward holds the wheel torque's reaction on the knee drive. */
    {
        double gravity[4];
        assert(bipe_feedforward(e.q, e.pitch, e.roll, gravity));
        assert(fabs(t6[4]) > 1e-3 && fabs(t6[5]) > 1e-3);
        for (unsigned j = 0; j < 4; j++) {
            double expected = gravity[j] - (j % 2 ? BIPE_KNEE_WHEEL_REACTION * t6[4 + j / 2] : 0);
            assert(near(ff[j], expected, 1e-12));
            assert(near(t6[j], fmax(-BIPE_JOINT_PEAK, fmin(BIPE_JOINT_PEAK,
                                    expected + BIPE_JOINT_KP * (posture[j] - e.q[j]) +
                                        BIPE_JOINT_KD * (qd[j] - e.qd[j]))), 1e-12));
        }
        assert(BIPE_KNEE_WHEEL_REACTION == 1.0); /* config/controller_m10.json */
    }
    /* Learned, chosen at READY: legs go to the policy's targets with no feedforward. */
    assert(!bipe_selector_choose(&d.selector, BIPE_CONTROLLER_LEARNED, BIPE_ACTIVE));
    assert(bipe_selector_choose(&d.selector, BIPE_CONTROLLER_LEARNED, BIPE_READY));
    fake_value = 0.1f; fake_calls = 0;
    assert(bipe_drive_step(&d, BIPE_READY, &e, &s, hub, 0, 0, posture, qd, 0.005, fake_forward, &out));
    assert(out.controller == BIPE_CONTROLLER_LEARNED && fake_calls == 1);
    /* Torque off (2 s in READY, wheels still, a large held action): the policy runs but
       its output stays at rest, so the frame of the arming tick (computed before the
       supervisor arms) is nominal legs, zero wheel speed and no feedforward. */
    fake_value = 0.5f;
    double rest[4];
    bipe_learned_nominal(bipe_learned_label(posture), rest);
    for (unsigned t = 0; t < 400; t++) {
        assert(bipe_drive_step(&d, BIPE_READY, &e, &s, hub, 0.4, 0, posture, qd, 0.005, fake_forward, &out));
        assert(out.controller == BIPE_CONTROLLER_LEARNED && out.wheel_speed_mode);
        assert(memcmp(out.qref, rest, sizeof rest) == 0 && out.qdref[0] == 0 && out.ff[3] == 0);
        assert(out.wheel_speed[0] == 0 && out.wheel_speed[1] == 0);
        assert(out.wheel_torque[0] == 0 && out.wheel_torque[1] == 0);
    }
    assert(fake_calls == 101 && d.learned.wheel_integral[0] == 0 && d.learned.action[0] == 0.5f);
    fake_value = 0.1f;
    /* Arming starts the policy afresh (zero held action, as in simulation). */
    assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0, 0, posture, qd, 0.005, fake_forward, &out));
    assert(d.learned.ticks == 1 && d.learned.observation[16] == 0 && fake_calls == 102);
    assert(d.learned.command[0] == 0 && near(d.learned.wheel_integral[0], 4.0 * 0.005, 1e-6));
    double expected[4];
    bipe_learned_nominal(d.learned.command[2], expected);
    /* The legs follow the targets through the leg target filter, one step a tick. */
    const double a = BIPE_LEARNED_TICK_S / (BIPE_LEARNED_LEG_FILTER_TAU_S + BIPE_LEARNED_TICK_S);
    for (unsigned j = 0; j < 4; j++) {
        assert(near(d.learned.leg_target[j], expected[j] + 0.5 * (j % 2 ? -0.1 : 0.1), 1e-8));
        assert(near(out.qref[j], expected[j] + a * 0.5 * (j % 2 ? -0.1 : 0.1), 1e-8));
        assert(out.qdref[j] == 0 && out.ff[j] == 0);
    }
    assert(out.wheel_speed_mode && out.wheel_speed[0] == d.learned.wheel_target[0]);
    assert(near(out.wheel_torque[1], BIPE_LEARNED_WHEEL_KI * d.learned.wheel_integral[1], 1e-15));
    /* A NaN action hands over in the same tick, to a fresh classical controller. */
    while (d.learned.ticks % 4)
        assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0, 0, posture, qd, 0.005, fake_forward, &out));
    fake_value = NAN;
    memset(&reference_balance, 0, sizeof reference_balance);
    for (unsigned t = 0; t < 10; t++) {
        assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0.2, 0, posture, qd, 0.005, fake_forward, &out));
        assert(bipe_control_step(&reference_balance, &e, 0.2, 0, posture, qd, 0.005, t6, ff, &pitch));
        assert(out.controller == BIPE_CONTROLLER_CLASSICAL);
        assert(out.wheel_torque[0] == t6[4] && memcmp(out.ff, ff, sizeof ff) == 0);
    }
    assert(d.selector.handovers == 1 && d.selector.locked_out);
    /* Stays classical after disarming and re-arming. */
    assert(!bipe_selector_choose(&d.selector, BIPE_CONTROLLER_LEARNED, BIPE_READY));
    assert(bipe_drive_step(&d, BIPE_READY, &e, &s, hub, 0, 0, posture, qd, 0.005, fake_forward, &out));
    assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0, 0, posture, qd, 0.005, fake_forward, &out));
    assert(out.controller == BIPE_CONTROLLER_CLASSICAL);

    /* Deferred forward pass (after the tick's commands): the action applies a tick later,
       and a NaN hands over from the next tick. */
    bipe_drive_init(&d, ok);
    assert(bipe_selector_choose(&d.selector, BIPE_CONTROLLER_LEARNED, BIPE_READY));
    fake_value = 0.1f; fake_calls = 0;
    assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0, 0, posture, qd, 0.005, NULL, &out));
    bipe_learned_nominal(d.learned.command[2], expected);
    assert(fake_calls == 0 && d.learned.due && memcmp(out.qref, expected, sizeof expected) == 0);
    assert(bipe_drive_forward(&d, fake_forward) && fake_calls == 1 && !d.learned.due);
    assert(!bipe_drive_forward(&d, fake_forward));
    assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0, 0, posture, qd, 0.005, NULL, &out));
    assert(near(out.qref[0], expected[0] + a * 0.05, 1e-8));
    assert(out.controller == BIPE_CONTROLLER_LEARNED && d.learned.action[0] == 0.1f);
    while (!d.learned.due)
        assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0, 0, posture, qd, 0.005, NULL, &out));
    fake_value = NAN;
    assert(bipe_drive_forward(&d, fake_forward) && d.selector.handovers == 1);
    assert(bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0, 0, posture, qd, 0.005, NULL, &out));
    assert(out.controller == BIPE_CONTROLLER_CLASSICAL);
    /* A non-finite input fails the step: the caller takes the protective path. */
    bipe_drive_init(&d, ok);
    assert(bipe_selector_choose(&d.selector, BIPE_CONTROLLER_LEARNED, BIPE_READY));
    s.gyro[0] = NAN;
    assert(!bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, hub, 0, 0, posture, qd, 0.005, fake_forward, &out));
    assert(!bipe_drive_step(&d, BIPE_ACTIVE, &e, NULL, hub, 0, 0, posture, qd, 0.005, fake_forward, &out));
    s.gyro[0] = 0;
    assert(!bipe_drive_step(&d, BIPE_ACTIVE, &e, &s, NULL, 0, 0, posture, qd, 0.005, fake_forward, &out));
    puts("learned: selection switch, posture table, command limits, observation layout, action law, "
         "leg/wheel clamps, non-finite input and output, handover (inline and deferred), knee wheel reaction pass");
    return 0;
}
