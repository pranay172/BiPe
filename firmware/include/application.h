#ifndef BIPE_APPLICATION_H
#define BIPE_APPLICATION_H
#include "bipe.h"
#include "follower.h"

/* Profile attestations are commissioning evidence, never inferred from a name.
   Synthetic profiles are admitted only in a separately compiled host test. */
enum bipe_app_evidence {
    BIPE_APP_BUS_LIMITS=1u<<0, BIPE_APP_CODEC=1u<<1,
    BIPE_APP_EFFORT_AND_GAINS=1u<<2, BIPE_APP_ENCODER_MAPPING=1u<<3,
    BIPE_APP_DRIVER_LIMITS=1u<<4, BIPE_APP_LOCAL_PD_AND_TIMEOUT=1u<<5,
    BIPE_APP_CAN_HARNESS=1u<<6, BIPE_APP_IMU_CALIBRATION=1u<<7,
    BIPE_APP_START_STOP_TIMING=1u<<8
};
#define BIPE_APP_REQUIRED_EVIDENCE 0x1ffu
typedef struct {
    bool synthetic;
    uint32_t evidence;
    bipe_ranges ranges;
    double output_Nm_per_effort, minimum_bus_V, maximum_bus_V;
    double encoder_sign[6], encoder_zero[6], imu_to_body[9];
    bool wheel_modulo_turn;
} bipe_app_profile;
typedef struct {
    /* send queues only qualified frames; disable must discard queued batches
       and disable retained local PD. Failure always removes power permission. */
    bool (*send)(void *context, const bipe_frame *frame);
    void (*power)(void *context, bool permission, bool heartbeat);
    void *context;
    bool (*disable_and_cancel_pending)(void *context);
} bipe_app_hal;
typedef struct {
    bipe_app_profile profile;
    bipe_app_hal hal;
    bipe_supervisor supervisor;
    bipe_inputs inputs;
    bipe_calibration calibration;
    bipe_estimator estimator;
    bipe_balance balance;
    bipe_estimate estimate;
    double gyro[3], accel[3], bias[3], wrapped[6], unwrapped[6];
    double velocity, yaw_rate, qref[4], qdref[4], last_effort[6];
    uint32_t last_control_ms, last_heartbeat_ms;
    unsigned sent_batches;
    uint8_t driver_limits_ack_mask;
    bool profile_valid, estimator_ready, control_seen, heartbeat, commands_started;
    /* M12 following: off unless the host requests it; a manual command, a fault or
       leaving ACTIVE cancels it. */
    bipe_follow_config follow_config;
    bipe_follower follower;
    double follow_velocity, follow_yaw_rate;
} bipe_application;
void bipe_app_init(bipe_application *a, const bipe_app_profile *profile, bipe_app_hal hal);
bool bipe_app_profile_authorized(const bipe_application *a);
void bipe_app_reset(bipe_application *a);
bool bipe_app_motor(bipe_application *a, const bipe_motor_sample *sample);
bool bipe_app_imu(bipe_application *a, const double gyro[3], const double accel[3],
                  uint32_t time_ms, uint32_t sequence);
bool bipe_app_command(bipe_application *a, double velocity, double yaw_rate,
                      const double qref[4], const double qdref[4], uint32_t now_ms);
void bipe_app_tick(bipe_application *a, uint32_t now_ms);
/* An explicit follow request (true) or cancel (false) from the host. A request zeroes the
   manual commands, so a cancelled follow stops rather than resuming an old command. */
bool bipe_app_follow_request(bipe_application *a, bool on, uint32_t now_ms);
/* One Pi report (a detection or an empty frame). While following is requested, a report
   that is not malformed keeps the host link alive. */
enum bipe_follow_report bipe_app_target(bipe_application *a, const bipe_target_report *r,
                                        uint32_t now_ms);
#endif
