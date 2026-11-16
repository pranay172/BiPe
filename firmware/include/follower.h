#ifndef BIPE_FOLLOWER_H
#define BIPE_FOLLOWER_H
#include <stdbool.h>
#include <stdint.h>

/* M12 camera following. The Pi sends one report per processed frame (a detection or an
   empty frame); the follower keeps a constant-velocity estimate of the target in the
   robot's odometry frame, fuses each report at its exposure time through a pose history,
   and returns velocity and yaw-rate commands for bipe_app_command. The supervisor and the
   firmware command gates keep the final say. SI units, radians. */
#define BIPE_FOLLOW_HISTORY 128u
#define BIPE_FOLLOW_TABLE 8u
enum bipe_follow_mode { BIPE_FOLLOW_OFF, BIPE_FOLLOW_WAITING, BIPE_FOLLOW_FOLLOWING,
                        BIPE_FOLLOW_COASTING, BIPE_FOLLOW_BRAKING, BIPE_FOLLOW_HOLDING };
enum bipe_follow_event { BIPE_FOLLOW_EVENT_NONE, BIPE_FOLLOW_EVENT_ENGAGED,
                         BIPE_FOLLOW_EVENT_COASTING, BIPE_FOLLOW_EVENT_LOST,
                         BIPE_FOLLOW_EVENT_REACQUIRED, BIPE_FOLLOW_EVENT_HOLDING,
                         BIPE_FOLLOW_EVENT_CANCELLED };
enum bipe_follow_report { BIPE_FOLLOW_ACCEPTED, BIPE_FOLLOW_EMPTY, BIPE_FOLLOW_INVALID,
                          BIPE_FOLLOW_REPEATED, BIPE_FOLLOW_WRONG_ID, BIPE_FOLLOW_TOO_OLD,
                          BIPE_FOLLOW_GATED, BIPE_FOLLOW_IGNORED };
typedef struct {
    uint32_t sequence, tag_id;
    bool detected;
    double range_m, bearing_rad, age_s; /* age: exposure middle to sending, from the Pi */
} bipe_target_report;
typedef struct {
    double dt, history_s, max_age_s, link_delay_s, frame_s, camera_ahead_m, camera_ahead_per_lean;
    double process_density, initial_velocity_sigma;
    unsigned table_n;
    double table_range[BIPE_FOLLOW_TABLE], table_range_std[BIPE_FOLLOW_TABLE],
           table_bearing_std[BIPE_FOLLOW_TABLE];
    double noise_scale, range_std_floor, bearing_std_floor;
    double innovation_chi2, gate_m;
    uint32_t tag_id;
    unsigned engage_detections;
    double engage_s, engage_range_min, engage_range_max, engage_speed_max;
    double coast_after_frames, coast_s, stale_s, hold_speed;
    double nominal_range, range_gain, bearing_gain, stop_range, lateral_range_floor;
    double speed_max, reverse_max, acceleration_max, yaw_rate_max, yaw_acceleration_max,
           brake_deceleration;
} bipe_follow_config;
typedef struct { double t, x, y, yaw, ahead, s[4], P[16]; } bipe_follow_sample;
typedef struct {
    enum bipe_follow_mode mode;
    bool requested, initialized, ticked, sequence_seen, message_seen;
    double s[4], P[16]; /* target [x, y, vx, vy] and covariance at the latest tick */
    bipe_follow_sample history[BIPE_FOLLOW_HISTORY];
    unsigned head, count; /* head: index of the latest sample */
    uint32_t last_sequence, streak_sequence;
    unsigned streak;
    double streak_start, last_range, last_fused, last_accepted, last_message, now;
    double velocity, yaw_rate, lost_x, lost_y;
    enum bipe_follow_event event, pending; /* the last step's event; one from a report,
                                              reported by the next step */
    unsigned reports[8]; /* counts by bipe_follow_report */
} bipe_follower;

bipe_follow_config bipe_follow_default_config(void);
bool bipe_follow_config_valid(const bipe_follow_config *c);
void bipe_follow_init(bipe_follower *f);
/* An explicit follow request starts waiting for the engage rule; false cancels. */
void bipe_follow_request(bipe_follower *f, bool on);
/* A report received at time now (the Teensy's clock). Returns how it was used. */
enum bipe_follow_report bipe_follow_report_in(bipe_follower *f, const bipe_follow_config *c,
    const bipe_target_report *r, double now);
/* One control tick: the robot's odometry pose, its body lean (pitch from the balanced
   equilibrium, positive forward; it moves the head camera along the body) and speed from
   the balance estimator, and
   whether the supervisor allows following (active, no fault). Returns false on invalid
   input. velocity and yaw_rate are written in every mode (zero when off). */
bool bipe_follow_step(bipe_follower *f, const bipe_follow_config *c, double now, double x,
    double y, double yaw, double lean, double robot_velocity, bool allowed, double *velocity,
    double *yaw_rate);
/* Range and bearing of the estimate from the camera at the latest tick (with its lean). */
bool bipe_follow_estimate(const bipe_follower *f, const bipe_follow_config *c,
    double *range, double *bearing);
#endif
