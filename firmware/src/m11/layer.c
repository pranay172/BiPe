#include "m11_layer.h"
#include "m11_hardware.h"
#include <math.h>
#include <string.h>

static const double TWO_PI = 6.283185307179586;
const bipe_m11_axis bipe_m11_axes[BIPE_MOTORS] = BIPE_M11_AXES;

unsigned bipe_lsm6dso_setup(uint8_t reg[4], uint8_t value[4]) {
    static const uint8_t R[4] = {0x12, 0x10, 0x11, 0x0d};
    static const uint8_t V[4] = {BIPE_M11_IMU_CTRL3_C, BIPE_M11_IMU_CTRL1_XL,
                                 BIPE_M11_IMU_CTRL2_G, BIPE_M11_IMU_INT1_CTRL};
    memcpy(reg, R, 4); memcpy(value, V, 4);
    return 4;
}
static void rotate(const double chip[3], double body[3]) {
    static const double R[9] = BIPE_M11_IMU_TO_BODY;
    for (unsigned i = 0; i < 3; i++)
        body[i] = R[3 * i] * chip[0] + R[3 * i + 1] * chip[1] + R[3 * i + 2] * chip[2];
}
void bipe_lsm6dso_body(const uint8_t raw[12], double gyro[3], double accel[3]) {
    double g[3], a[3];
    for (unsigned i = 0; i < 3; i++) {
        int16_t gi = (int16_t)((uint16_t)raw[2 * i] | (uint16_t)raw[2 * i + 1] << 8);
        int16_t ai = (int16_t)((uint16_t)raw[6 + 2 * i] | (uint16_t)raw[7 + 2 * i] << 8);
        g[i] = gi * BIPE_M11_IMU_GYRO_RAD_S_PER_LSB;
        a[i] = ai * BIPE_M11_IMU_ACCEL_M_S2_PER_LSB;
    }
    rotate(g, gyro); rotate(a, accel);
}
bool bipe_m11_gravity_ok(const double a[3]) {
    double norm = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    return isfinite(norm) && norm > 0 && a[2] / norm >= cos(BIPE_M11_IMU_GRAVITY_CHECK_RAD);
}

static bool position(const bipe_m11_axis *a, const double value[5], const uint8_t res[5],
                     bipe_fd_frame *f) {
    if (!a || !f) return false;
    bipe_mx_position c = bipe_mx_empty_position();
    for (unsigned i = 0; i < 5; i++) { c.value[i] = value[i]; c.resolution[i] = res[i]; }
    return bipe_mx_position_frame(a->id, BIPE_M11_CAN_SOURCE, &c, NULL, f);
}
bool bipe_m11_joint_command(const bipe_m11_axis *a, double qref, double qdref, double ff,
                            bipe_fd_frame *f) {
    if (!a || a->wheel || !isfinite(qref) || !isfinite(qdref) || !isfinite(ff)) return false;
    const double v[5] = {a->sign * (qref - a->centre_rad) / TWO_PI, a->sign * qdref / TWO_PI,
                         a->sign * ff, 0, 0};
    const uint8_t r[5] = {BIPE_MX_F32, BIPE_MX_F32, BIPE_MX_F32, BIPE_MX_IGNORE,
                          BIPE_MX_IGNORE};
    return position(a, v, r, f);
}
bool bipe_m11_wheel_command(const bipe_m11_axis *a, double torque, bipe_fd_frame *f) {
    if (!a || !a->wheel || !isfinite(torque)) return false;
    const double v[5] = {NAN, 0, a->sign * torque, 0, 0};
    const uint8_t r[5] = {BIPE_MX_F32, BIPE_MX_F32, BIPE_MX_F32, BIPE_MX_F32, BIPE_MX_F32};
    return position(a, v, r, f);
}
bool bipe_m11_wheel_speed_command(const bipe_m11_axis *a, double speed, double ff,
                                  bipe_fd_frame *f) {
    if (!a || !a->wheel || !isfinite(speed) || !isfinite(ff)) return false;
    const double v[5] = {NAN, a->sign * speed / TWO_PI, a->sign * ff, 0, 1};
    const uint8_t r[5] = {BIPE_MX_F32, BIPE_MX_F32, BIPE_MX_F32, BIPE_MX_F32, BIPE_MX_F32};
    return position(a, v, r, f);
}
bool bipe_m11_query(const bipe_m11_axis *a, bipe_fd_frame *f) {
    if (!a) return false;
    bipe_mx_query q = bipe_mx_default_query();
    return bipe_mx_query_frame(a->id, BIPE_M11_CAN_SOURCE, &q, f);
}
bool bipe_m11_stop(const bipe_m11_axis *a, bipe_fd_frame *f) {
    return a && bipe_mx_stop_frame(a->id, BIPE_M11_CAN_SOURCE, NULL, f);
}
bool bipe_m11_sample(const bipe_m11_axis *a, const bipe_fd_frame *f, uint32_t now,
                     uint32_t sequence, bipe_motor_sample *s) {
    static const uint16_t NEEDED = 1u << BIPE_MX_MODE | 1u << BIPE_MX_POSITION_REG |
        1u << BIPE_MX_VELOCITY | 1u << BIPE_MX_TEMPERATURE | 1u << BIPE_MX_FAULT_CODE;
    bipe_mx_reply r;
    if (!a || !s || !bipe_mx_parse(f, BIPE_M11_CAN_SOURCE, &r) || r.source != a->id ||
        (r.present & NEEDED) != NEEDED)
        return false;
    double turns = r.value[BIPE_MX_POSITION_REG], rate = r.value[BIPE_MX_VELOCITY];
    int mode = (int)r.value[BIPE_MX_MODE], code = (int)r.value[BIPE_MX_FAULT_CODE];
    memset(s, 0, sizeof *s);
    s->present = true;
    s->valid = isfinite(turns) && isfinite(rate);
    s->time_ms = now; s->sequence = sequence; s->id = a->id;
    s->position = (a->wheel ? 0.0 : a->centre_rad) + a->sign * TWO_PI * turns;
    s->speed = a->sign * TWO_PI * rate;
    s->temperature = r.value[BIPE_MX_TEMPERATURE];
    if (mode == BIPE_MX_FAULT || mode == BIPE_MX_TIMEOUT)
        s->error = (uint8_t)(code > 0 && code < 256 ? code : mode);
    return true;
}
double bipe_m11_bus_volts(unsigned counts) {
    return counts * (BIPE_M11_ADC_REFERENCE_V / BIPE_M11_ADC_MAX_COUNTS) * BIPE_M11_BUS_ADC_SCALE;
}
bipe_m11_outputs bipe_m11_outputs_for(const bipe_supervisor *s, bool qualified,
                                      bool power_good) {
    bipe_m11_outputs o = {false, false, false};
    if (!s || s->state == BIPE_OFF || !power_good) return o;
    o.transceiver_on = true;
    o.commands = qualified && s->torque_enabled;
    o.stop_frames = !o.commands;
    return o;
}
