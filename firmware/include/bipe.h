#ifndef BIPE_H
#define BIPE_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define BIPE_MOTORS 6
enum bipe_state { BIPE_OFF, BIPE_DISCOVERING, BIPE_CALIBRATING, BIPE_READY, BIPE_ACTIVE,
                  BIPE_STOPPING, BIPE_FAULT, BIPE_FALLING, BIPE_FALLEN };
enum bipe_fault { BIPE_OK, BIPE_PROFILE, BIPE_IMU, BIPE_FEEDBACK, BIPE_BUS_OFF,
                  BIPE_VOLTAGE, BIPE_TEMPERATURE, BIPE_ESTOP, BIPE_DEADLINE,
                  BIPE_ATTITUDE, BIPE_ENCODER, BIPE_DRIVER, BIPE_HOST };
typedef struct { uint32_t id; uint8_t dlc, data[8]; bool extended, remote; } bipe_frame;
typedef struct { double lo, hi; } bipe_range;
typedef struct { bipe_range kp, kd, position, velocity, effort; } bipe_ranges;
typedef struct { double kp, kd, position, velocity, effort; } bipe_mit;
typedef struct { uint8_t id, error; int8_t temperature_C;
                 double position_rad, velocity_rad_s, iq_A; } bipe_feedback;
bool bipe_pack(uint8_t id, const bipe_ranges *ranges, const bipe_mit *cmd,
               bipe_frame *frame);
bool bipe_decode(const bipe_frame *frame, unsigned pole_pairs, double reduction,
                 bipe_feedback *feedback);
bipe_frame bipe_disable(uint8_t id);
/* Modulo-one-turn reconstruction ONLY for profiles that establish that convention.
   Never use this function to hide a stale sample or an encoder discontinuity. */
bool bipe_unwrap(double previous_wrapped, double wrapped, double previous_total,
                 double elapsed_s, double maximum_speed, double *total);

typedef struct {
    bool present, valid;
    uint32_t time_ms, sequence;
    double position, speed, temperature;
    uint8_t id, error;
} bipe_motor_sample;
typedef struct {
    bool imu_valid, imu_present, bus_off, estop_closed, power_good;
    uint32_t imu_time_ms, imu_sequence, host_time_ms;
    double gyro_norm, acceleration_norm, attitude_error, bus_V;
    bipe_motor_sample motor[BIPE_MOTORS];
} bipe_inputs;
typedef struct {
    unsigned feedback_ms, imu_ms, host_ms, stop_ms, deadline_ms, calibration_ms;
    double bus_arm_min, bus_arm_max, bus_fault_min, bus_fault_max, temperature_max;
    double stationary_gyro, stationary_joint_speed, attitude_max;
} bipe_settings;
typedef struct {
    enum bipe_state state;
    enum bipe_fault fault;
    uint32_t last_tick, calibration_since, stop_since, last_imu_sequence;
    unsigned calibration_samples;
    bool tick_seen, calibrating_stationary, start_requested, arm_requested;
    bool power_request, torque_enabled;
    unsigned fall_stop_ms;
} bipe_supervisor;
void bipe_init(bipe_supervisor *s);
bipe_settings bipe_default_settings(void);
void bipe_start(bipe_supervisor *s);
void bipe_arm(bipe_supervisor *s);
void bipe_reset(bipe_supervisor *s);
/* M9 only: a bounded protective interval retains all health checks except the
   already-detected attitude excursion. Torque-off remains latched until reset. */
bool bipe_begin_fall(bipe_supervisor *s,uint32_t now_ms,unsigned maximum_ms);
void bipe_end_fall(bipe_supervisor *s);
void bipe_tick(bipe_supervisor *s, const bipe_settings *settings,
               const bipe_inputs *in, uint32_t now_ms, bool profile_qualified);

typedef struct {
    double pitch, pitch_rate, yaw, yaw_rate, x, y, velocity;
    double equilibrium, height_scale, wheel_speed[2];
    double q[4], qd[4], qref[4], qdref[4];
} bipe_observation;
typedef struct {
    double integral, vref, hold_x, hold_y, previous_velocity, heading;
    bool hold_initialized;
    double common_lag;
    /* Optional slope lean (BIPE_SLOPE_OBSERVER_TAU_S > 0): whole-robot momentum observer. */
    double slope_momentum, slope_force, wheel_sum, slope_lean, slope_offset; /* wheel_sum: before
       friction compensation, which the hub friction absorbs */
    bool slope_initialized;
    /* Optional jerk-limited reference (BIPE_COMMAND_JERK_M_S3 > 0): its acceleration, the lean
       feedforward atan(a/g) and the brake-to-rest flag of a stop command. */
    double command_acceleration, acceleration_lean;
    bool braking;
} bipe_balance;
/* Accepted M3 wheel supervisor, SI inputs, finite wheel effort. */
bool bipe_balance_step(bipe_balance *b, const bipe_observation *o, double velocity,
                       double yaw_rate, double dt, double wheel_Nm[2]);
/* State estimator input is a sampled body-frame IMU and six unwrapped SI encoders.
   Transport delay/device bandwidth are modeled at the sensor boundary. */
typedef struct { double dt, velocity_tau, joint_tau, posture_tau, correction_tau,
    accel_gate, prediction_delay, local_tau, gravity, wheel_radius;
    /* Optional IMU-fused odometry, serial profile only; fusion_tau 0 disables it. */
    double fusion_tau, innovation_limit, bias_tau; } bipe_sensor_config;
typedef struct { double pitch, roll, yaw, x, y, velocity, previous[6], posture[4], rates[6];
    bool initialized;
    double imu_velocity, accel_bias, imu_offset; bool fusion_ready;
    /* With the fusion: the IMU point's forward acceleration from odometry, taken out of the
       accelerometer before the attitude correction, clipped to compensation_limit. That
       limit is an input the caller sets after each control step: the drive acceleration
       limit while driving (a drive command or a brake to rest), 0 while holding, so a push
       is not mistaken for a drive acceleration. */
    double imu_acceleration, odometry, compensation_limit; } bipe_estimator;
typedef struct { double pitch, pitch_rate, roll, yaw, yaw_rate, x, y, velocity,
    q[4], qd[4], wheel_speed[2], accel_norm; } bipe_estimate;
typedef struct { double previous[4], rates[4]; bool initialized; } bipe_local_feedback;
typedef struct { double gyro_sum[3], accel_sum[3]; unsigned samples;
    uint32_t since_ms, last_ms, last_sequence; bool started; } bipe_calibration;
bipe_sensor_config bipe_default_sensor_config(void);
bool bipe_estimator_init(bipe_estimator *s, const double gravity_sample[3], const double encoder[6]);
bool bipe_estimator_step(bipe_estimator *s, const bipe_sensor_config *config,
    const double gyro[3], const double accel[3], const double encoder[6], bipe_estimate *out);
bool bipe_local_step(bipe_local_feedback *s, const double q[4], double dt, double tau, double rates[4]);
/* Reset an interrupted stationary calibration; only fresh samples count. */
bool bipe_calibrate(bipe_calibration *s, const double gyro[3], const double accel[3],
    bool encoders_stationary, uint32_t now_ms, uint32_t sequence, double bias[3], double gravity_sample[3]);
/* Hardware encoder mapping is supplied by a qualified profile, never guessed. */
bool bipe_map_joints(const double raw[4], const double sign[4], const double zero[4], double mapped[4]);
bool bipe_joint_limits(const double q[4], double tolerance);
/* Inherited posture degrees; M9 maps to SI height and native five-bar IK. */
bool bipe_posture(double angle_deg, double q[4]);
bool bipe_kinematics(const double q[4], double com[2], double axle[2], double *equilibrium, double *height_scale);
/* Wheel-carrier CAD-angle rates (opposite +Y) from output-shaft encoders. */
bool bipe_carrier_rates(const double q[4], const double qd[4], double rates[2]);
bool bipe_feedforward(const double q[4], double pitch, double roll, double torque[4]);
bool bipe_legs(const double q[4], const double qd[4], const double qref[4],
    const double qdref[4], const double feedforward[4], double torque[4]);
bool bipe_control_step(bipe_balance *s, const bipe_estimate *e, double velocity,
    double yaw_rate, const double qref[4], const double qdref[4], double dt,
    double torque[6], double feedforward[4], double *pitch_reference);
enum bipe_fall_reason { BIPE_FALL_NONE, BIPE_FALL_PITCH, BIPE_FALL_ROLL,
    BIPE_FALL_FREEFALL, BIPE_FALL_INVALID };
typedef struct { double pitch_rad,hard_pitch_rad,roll_rad,outward_rate_rad_s,persist_s,
    freefall_m_s2,freefall_persist_s,posture_deg,reference_rate_rad_s,crouch_s,ramp_s,
    wheel_ramp_s,joint_cap_Nm; } bipe_fall_settings;
typedef struct { double exposure_s,freefall_s,age_s,qref[4],qdref[4],wheel[2];
    unsigned reason; bool latched,torque_off; } bipe_fall;
bool bipe_fall_step(bipe_fall *s,const bipe_fall_settings *c,const bipe_estimate *e,
    bool valid,double dt,const double previous[6],double torque[6]);
bool bipe_fall_legs(const bipe_fall *s,const bipe_fall_settings *c,const double q[4],
    const double qd[4],double torque[4]);
#endif
