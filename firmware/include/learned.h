/* M13 learned controller on the robot: the selection switch, the policy's runtime around
   the forward pass (locomotion.h) and its output guards, and the drive step that
   composes it with the classical controller. Portable C; docs/milestones/
   13-learned-locomotion-plan.md, "Hardware wiring checklist". */
#ifndef BIPE_LEARNED_H
#define BIPE_LEARNED_H
#include "bipe.h"
#include "locomotion.h"

/* Selection switch. The build sets the default (classical unless the image is built with
   -DBIPE_CONTROLLER_DEFAULT=BIPE_CONTROLLER_LEARNED); a choice is taken only while torque
   is off (OFF to READY) and holds for the whole armed run. The learned controller is
   selectable only when the linked forward pass reports the checkpoint the image's
   locomotion_config.h was generated for. A handover locks it out until init. */
enum bipe_controller { BIPE_CONTROLLER_CLASSICAL, BIPE_CONTROLLER_LEARNED };
#ifndef BIPE_CONTROLLER_DEFAULT
#define BIPE_CONTROLLER_DEFAULT BIPE_CONTROLLER_CLASSICAL
#endif
typedef struct {
    enum bipe_controller chosen, running;
    bool learned_available, locked_out;
    unsigned handovers;
} bipe_selector;
/* checkpoint: bipe_locomotion_checkpoint() of the linked forward pass, NULL if none. */
void bipe_selector_init(bipe_selector *s, const char *checkpoint);
bool bipe_selector_choose(bipe_selector *s, enum bipe_controller c, enum bipe_state state);
/* Each tick: the controller for this tick. The choice is taken up while torque is off. */
enum bipe_controller bipe_selector_update(bipe_selector *s, enum bipe_state state);
void bipe_selector_handover(bipe_selector *s);

/* The policy's runtime (bipe.m13.policy_control.PolicyController in C). */
enum bipe_learned_status { BIPE_LEARNED_OK, BIPE_LEARNED_BAD_INPUT, BIPE_LEARNED_BAD_OUTPUT };
typedef void (*bipe_forward_fn)(const float observation[BIPE_LOCOMOTION_OBSERVATIONS],
                                float action[BIPE_LOCOMOTION_ACTIONS]);
/* The latest body-frame IMU sample the estimator was given, the raw leg encoders, and
   the estimator's leg joint and wheel speeds (manifest observation layout). */
typedef struct { double gyro[3], accel[3], q[4], qd[4], wheel_speed[2]; } bipe_learned_sensors;
typedef struct {
    double command[3];       /* rate-limited speed m/s, yaw rate rad/s, posture label */
    float observation[BIPE_LOCOMOTION_OBSERVATIONS], action[BIPE_LOCOMOTION_ACTIONS];
    double leg_target[4], wheel_target[2], wheel_integral[2];
    double leg_output[4];    /* leg_target through the leg target filter: to the drives */
    unsigned ticks, leg_clamps, wheel_clamps; /* clamp counts since init (this run) */
    bool due;                /* an observation waits for the forward pass */
} bipe_learned;
void bipe_learned_nominal(double label, double q[4]);
/* The posture label whose nominal references are nearest qref (interpolated). */
double bipe_learned_label(const double qref[4]);
/* Starts at rest at the posture of qref, with zero action and wheel integral. */
void bipe_learned_init(bipe_learned *l, const double qref[4]);
/* Every control tick. On a policy tick (every BIPE_LEARNED_POLICY_TICKS) it rate-limits
   the command (clamped to the trained ranges) and builds the observation, then waits for
   bipe_learned_forward. BAD_INPUT if a sensor value is not finite (nothing changes). */
enum bipe_learned_status bipe_learned_tick(bipe_learned *l, const bipe_learned_sensors *s,
                                           double velocity, double yaw_rate,
                                           const double qref[4]);
/* Runs the forward pass on the waiting observation. A finite action becomes the held
   action and the leg and wheel targets, clamped to the joint limits and the hub speed;
   a non-finite one is BAD_OUTPUT and changes nothing. Called straight after the tick, the
   action applies in that tick, as in simulation; called after the tick's commands are
   sent, it applies from the next tick (one tick later), off the sample-to-actuation path. */
enum bipe_learned_status bipe_learned_forward(bipe_learned *l, bipe_forward_fn forward);
/* Every control tick, after the forward pass: one step of the first-order filter
   (time constant BIPE_LEARNED_LEG_FILTER_TAU_S) from leg_target to leg_output, the leg
   references the drives servo to (time constant 0: off, leg_output = leg_target), kept within BIPE_LEARNED_LEG_FILTER_BAND_RAD of the
   target so large moves (push reactions) pass with at most that lag. It damps the
   knee-belt mode at the soft end of the belt range without the step-to-step action
   chatter more joint damping causes at the stiff end (M13 plan, "Belt range follow-up"). */
void bipe_learned_legs(bipe_learned *l, double q[4]);
/* Wheel speed loop on the hubs' own speed, clamped to the hub torque. */
bool bipe_learned_wheels(bipe_learned *l, const double hub_speed[2], double dt,
                         double torque[2]);

/* Drive step: the selected controller for one 5 ms tick. The learned controller hands
   over to the classical one (fresh balance state, for the rest of the run) when its
   output is not finite; a non-finite input is a sensing fault, so the step fails and
   the caller takes the supervisor's protective path, as for a failed classical step.
   While torque is off the selected controller still runs (so its time is measured), but
   the learned output is held at rest (nominal legs, still wheels, no feedforward): the
   frames of the tick that arms are computed before the supervisor arms. Both
   controllers start afresh on the first tick with torque, as in simulation. */
typedef struct {
    bipe_selector selector;
    bipe_learned learned;
    bipe_balance balance;
    enum bipe_controller previous;
    bool torque;
} bipe_drive;
typedef struct {
    enum bipe_controller controller;
    /* Legs: drive-local PD to qref, qdref with feedforward ff. Wheels: torque, or with
       wheel_speed_mode (learned) the speed target for the drive's own kd (the policy's
       kv, at the drive's rate) with wheel_torque the speed loop's integral feedforward:
       the 5 ms tick alone is too slow for the trained 1 kHz loop. */
    double qref[4], qdref[4], ff[4], wheel_torque[2], wheel_speed[2];
    bool wheel_speed_mode;
} bipe_drive_output;
void bipe_drive_init(bipe_drive *d, const char *checkpoint);
/* forward NULL: the caller runs bipe_drive_forward after sending the tick's commands. */
bool bipe_drive_step(bipe_drive *d, enum bipe_state state, const bipe_estimate *e,
                     const bipe_learned_sensors *s, const double hub_speed[2],
                     double velocity, double yaw_rate, const double qref[4],
                     const double qdref[4], double dt, bipe_forward_fn forward,
                     bipe_drive_output *out);
/* Runs a waiting forward pass; hands over on BAD_OUTPUT. True if the pass ran. */
bool bipe_drive_forward(bipe_drive *d, bipe_forward_fn forward);

/* Bench parity check (src/locomotion/parity.c, with the export's generated parity
   vectors): the forward pass on recorded observations against the float64 actor, each
   call timed with cycles() when given (the Teensy's DWT cycle counter). */
typedef struct {
    unsigned vectors;
    double max_error;
    uint32_t min_cycles, max_cycles;
    bool passed; /* every action within the parity.json tolerance, 1e-4, and the vectors'
                    checkpoint is the linked weights' */
} bipe_parity_result;
bipe_parity_result bipe_locomotion_parity(bipe_forward_fn forward, uint32_t (*cycles)(void));
#endif
