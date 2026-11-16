#ifndef BIPE_FALLEN_H
#define BIPE_FALLEN_H
#include "bipe.h"
#include <stdbool.h>
#include <stdint.h>

/* M16: the estimator's fallen mode. The balance estimator (bipe_estimator_step) refuses
   states it cannot represent: legs outside the range its kinematics accept, as in a
   fallen pose, and a pitch within 6 degrees of vertical. While it refuses, this wrapper
   returns the raw channels instead (attitude from the accelerometer, body rates from the
   gyro, leg and wheel rates from filtered encoder differences, no odometry) and
   re-initialises the estimator from that packet. M17: the estimator takes over only after
   BIPE_FALLEN_TAKEOVER_TICKS quiet ticks in a row that it accepts. `accepted` counts
   consecutive accepted ticks, the get-up handover's settling test. M16's
   bipe.m15.getup_eval modelled the M16 rule in Python (--fallen python). */

/* M17: body rate (rad/s, each gyro axis) under which a packet counts as quiet for the
   estimator to take over; the get-up handover's pitch-rate limit. */
#define BIPE_FALLEN_TAKEOVER_RATE 1.0
/* Consecutive quiet ticks before the estimator takes over (100 ms at the 5 ms tick). */
#define BIPE_FALLEN_TAKEOVER_TICKS 20u

typedef struct {
    bipe_estimator estimator;
    double previous[6], rates[6];
    uint32_t accepted, refused, quiet;
    bool started;
} bipe_fallen_estimator;

/* Returns false only for invalid inputs (non-finite samples or settings). *valid tells
   whether the balance estimator accepted this tick (out then holds its estimate). */
bool bipe_fallen_step(bipe_fallen_estimator *s, const bipe_sensor_config *config,
    const double gyro[3], const double accel[3], const double encoder[6],
    bipe_estimate *out, bool *valid);
#endif
