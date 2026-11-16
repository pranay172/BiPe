#ifndef BIPE_M11_LAYER_H
#define BIPE_M11_LAYER_H
#include "bipe.h"
#include "m11_moteus.h"

/* M11 hardware layer: LSM6DSO samples in the body frame, moteus frames from controller
   outputs, driver replies as supervisor samples, and hardware outputs per supervisor
   state. Portable C; the Teensy image supplies SPI, CAN and pins. */

/* LSM6DSO setup writes in order (CTRL3_C first: BDU and auto-increment). Returns 4. */
unsigned bipe_lsm6dso_setup(uint8_t reg[4], uint8_t value[4]);
/* raw: the 12-byte burst from OUTX_L_G (22h), gyro X, Y, Z then accel X, Y, Z as
   little-endian int16. Outputs rad/s and m/s^2 in the body frame. */
void bipe_lsm6dso_body(const uint8_t raw[12], double gyro[3], double accel[3]);
/* Startup check: the specific force lies within the configured angle of body +z. */
bool bipe_m11_gravity_ok(const double accel[3]);

/* One driver: joint q = centre + sign 2 pi position_rev; wheel q = sign 2 pi position_rev. */
typedef struct { uint8_t id; bool wheel; double sign, centre_rad; } bipe_m11_axis;
extern const bipe_m11_axis bipe_m11_axes[BIPE_MOTORS];
/* Joint: position mode with position, velocity and feedforward (f32); the moteus local
   PD uses its configured kp, kd. Wheel: feedforward torque only (position unset, kp and
   kd scale 0). Neither requests a reply; feedback comes from the query phase. */
bool bipe_m11_joint_command(const bipe_m11_axis *a, double qref, double qdref, double ff_Nm,
                            bipe_fd_frame *frame);
bool bipe_m11_wheel_command(const bipe_m11_axis *a, double torque_Nm, bipe_fd_frame *frame);
/* Wheel speed command (the M13 learned controller): velocity with kd scale 1, so the drive
   runs the configured kd (the policy's kv) at its own rate, plus feedforward torque. */
bool bipe_m11_wheel_speed_command(const bipe_m11_axis *a, double speed_rad_s, double ff_Nm,
                                  bipe_fd_frame *frame);
bool bipe_m11_query(const bipe_m11_axis *a, bipe_fd_frame *frame);
bool bipe_m11_stop(const bipe_m11_axis *a, bipe_fd_frame *frame);
/* A driver reply as a supervisor sample: SI position and speed, board temperature, and
   error = the fault code (or the mode) when the driver is in fault or timeout mode. */
bool bipe_m11_sample(const bipe_m11_axis *a, const bipe_fd_frame *frame, uint32_t now_ms,
                     uint32_t sequence, bipe_motor_sample *sample);
double bipe_m11_bus_volts(unsigned adc_counts);
/* Hardware outputs per supervisor state. The transceiver leaves standby once the
   supervisor starts and the LM5060 reports power good (the drivers are powered); torque
   frames go out only while it enables torque (active, stopping, falling) and the profile
   is qualified; every other running state sends stop frames. */
typedef struct { bool transceiver_on, commands, stop_frames; } bipe_m11_outputs;
bipe_m11_outputs bipe_m11_outputs_for(const bipe_supervisor *s, bool qualified,
                                      bool power_good);
#endif
