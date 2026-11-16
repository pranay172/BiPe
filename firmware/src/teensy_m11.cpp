/* M11 bench image: CAN3 CAN-FD to six moteus-c1 drivers, the LSM6DSO on SPI and the
   carrier's power and stop inputs, on the 200 Hz cycle of artifacts/m11/firmware/timing.json.
   The torque path is locked: the M10 profile is unqualified for hardware, so the
   supervisor never enables torque and the drivers only see queries and stop frames.
   This is a bench image for commissioning measurements, not a qualified application.
   Controllers (learned.h): classical by default (build flag BIPE_CONTROLLER_DEFAULT),
   chosen over serial while torque is off ('c' classical, 'l' learned); the policy's
   forward pass runs after the tick's commands are sent and is DWT-timed; 'p' runs the
   bench parity check (before start). The forward pass and parity vectors need
   python -m bipe.m13.export's generated headers. */
#include <Arduino.h>
#include <SPI.h>
#include <FlexCAN_T4.h>
extern "C" {
#include "bipe.h"
#include "m11_layer.h"
#include "m11_hardware.h"
#include "generated_config.h"
#include "learned.h"
}

static_assert(BIPE_SERIAL == 1, "The M11 image compiles against the M10 generated_config.h");
static_assert(BIPE_HARDWARE_QUALIFIED == 0,
              "M11 bench image: a torque path needs a separately reviewed, qualified image");

static FlexCAN_T4FD<CAN3, RX_SIZE_64, TX_SIZE_16> can;
static constexpr uint32_t CYCLE_US = 5000;  /* plant control_dt */
static constexpr uint32_t FEEDBACK_US = 1500; /* timing.json driver_feedback_s, rounded up */

static bipe_supervisor supervisor;
static bipe_inputs inputs = {};
static bipe_calibration calibration = {};
static bipe_estimator estimator = {};
static bipe_estimate estimate = {};
static bipe_drive drive;
static bool estimate_ready = false, can_ok = false, imu_ok = false, previous_arm = false;
static double gyro[3] = {}, accel[3] = {}, gyro_bias[3] = {};
static uint32_t imu_us = 0, last_cycle = 0, last_report = 0;
/* DWT cycle counts per phase: worst case since the last report. */
static uint32_t worst_feedback = 0, worst_compute = 0, worst_commands = 0, worst_policy = 0,
                missing = 0;

static const SPISettings IMU_SPI(BIPE_M11_IMU_SPI_HZ, MSBFIRST, SPI_MODE3);

static uint8_t imu_read(uint8_t reg) {
    SPI.beginTransaction(IMU_SPI);
    digitalWrite(BIPE_M11_PIN_IMU_CS, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t value = SPI.transfer(0);
    digitalWrite(BIPE_M11_PIN_IMU_CS, HIGH);
    SPI.endTransaction();
    return value;
}
static void imu_write(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(IMU_SPI);
    digitalWrite(BIPE_M11_PIN_IMU_CS, LOW);
    SPI.transfer(reg & 0x7f);
    SPI.transfer(value);
    digitalWrite(BIPE_M11_PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}
/* INT1 is the gyro data-ready (INT1_CTRL); the burst from OUTX_L_G clears it. */
static void imu_poll() {
    if (!imu_ok || digitalReadFast(BIPE_M11_PIN_IMU_INT1) == LOW) return;
    uint8_t raw[12];
    SPI.beginTransaction(IMU_SPI);
    digitalWrite(BIPE_M11_PIN_IMU_CS, LOW);
    SPI.transfer(BIPE_M11_IMU_FIRST_OUTPUT | 0x80);
    for (auto &b : raw) b = SPI.transfer(0);
    digitalWrite(BIPE_M11_PIN_IMU_CS, HIGH);
    SPI.endTransaction();
    bipe_lsm6dso_body(raw, gyro, accel);
    imu_us = micros();
    inputs.imu_sequence++;
}

static bool send(const bipe_fd_frame &f) {
    CANFD_message_t m;
    m.id = f.id; m.flags.extended = 1; m.brs = 1; m.edl = 1; m.len = f.size;
    memcpy(m.buf, f.data, f.size);
    return can.write(m) > 0;
}
static bool receive(bipe_fd_frame &f) {
    CANFD_message_t m;
    if (!can.read(m) || !m.flags.extended || m.len > BIPE_FD_MAX) return false;
    f.id = m.id; f.size = m.len;
    memcpy(f.data, m.buf, m.len);
    return true;
}

/* Query all six, then collect replies until each driver answered or the phase ends. */
static void feedback(uint32_t now_ms) {
    bipe_fd_frame f;
    for (const auto &a : bipe_m11_axes)
        if (bipe_m11_query(&a, &f)) send(f);
    unsigned answered = 0;
    const uint32_t start = micros();
    while (answered != (1u << BIPE_MOTORS) - 1 && micros() - start < FEEDBACK_US) {
        imu_poll();
        if (!receive(f)) continue;
        for (unsigned j = 0; j < BIPE_MOTORS; j++)
            if (bipe_m11_sample(&bipe_m11_axes[j], &f, now_ms, inputs.imu_sequence,
                                &inputs.motor[j]))
                answered |= 1u << j;
    }
    if (answered != (1u << BIPE_MOTORS) - 1) missing++;
}

/* Estimator and controller at the M10 rates; the result never reaches the drivers in
   this image (stop frames below), so the cycle can be timed on the bench. */
static void compute(uint32_t now_ms, bipe_fd_frame frames[BIPE_MOTORS]) {
    double encoder[6];
    bool stationary = true;
    for (unsigned j = 0; j < BIPE_MOTORS; j++) {
        encoder[j] = inputs.motor[j].position;
        stationary = stationary && inputs.motor[j].present &&
                     fabs(inputs.motor[j].speed) < BIPE_STATIONARY_JOINT_SPEED;
    }
    double g2 = 0, a2 = 0;
    for (unsigned i = 0; i < 3; i++) { g2 += gyro[i] * gyro[i]; a2 += accel[i] * accel[i]; }
    inputs.gyro_norm = sqrt(g2);
    inputs.acceleration_norm = sqrt(a2);
    double qref[4], qdref[4] = {};
    bipe_posture(40.0, qref);
    bipe_drive_output out = {};
    memcpy(out.qref, qref, sizeof qref);
    if (!estimate_ready) {
        double gravity[3];
        if (bipe_calibrate(&calibration, gyro, accel, stationary, now_ms, inputs.imu_sequence,
                           gyro_bias, gravity))
            estimate_ready = bipe_estimator_init(&estimator, gravity, encoder);
        inputs.attitude_error = 0.0;
    } else {
        double corrected[3];
        for (unsigned i = 0; i < 3; i++) corrected[i] = gyro[i] - gyro_bias[i];
        const bipe_sensor_config cfg = bipe_default_sensor_config();
        /* The policy observes the sample the estimator is given, the raw leg encoders
           and the estimator's speeds; the wheel speed loop runs on the hubs' own speed. */
        bipe_learned_sensors sensors = {};
        memcpy(sensors.gyro, corrected, sizeof corrected);
        memcpy(sensors.accel, accel, sizeof accel);
        memcpy(sensors.q, encoder, sizeof sensors.q);
        const double hub[2] = {inputs.motor[4].speed, inputs.motor[5].speed};
        /* A failed step (a non-finite input, or the classical step) is the supervisor's
           protective path: the attitude check below faults it. */
        estimate_ready = bipe_estimator_step(&estimator, &cfg, corrected, accel, encoder,
                                             &estimate);
        if (estimate_ready) {
            memcpy(sensors.qd, estimate.qd, sizeof sensors.qd);
            memcpy(sensors.wheel_speed, estimate.wheel_speed, sizeof sensors.wheel_speed);
            estimate_ready = bipe_drive_step(&drive, supervisor.state, &estimate, &sensors, hub,
                                             0, 0, qref, qdref, CYCLE_US * 1e-6, NULL, &out);
        }
        inputs.attitude_error = estimate_ready ? estimate.pitch : 1.0;
    }
    for (unsigned j = 0; j < 4; j++)
        bipe_m11_joint_command(&bipe_m11_axes[j], out.qref[j], out.qdref[j], out.ff[j], &frames[j]);
    for (unsigned j = 4; j < BIPE_MOTORS; j++) {
        if (out.wheel_speed_mode)
            bipe_m11_wheel_speed_command(&bipe_m11_axes[j], out.wheel_speed[j - 4],
                                         out.wheel_torque[j - 4], &frames[j]);
        else
            bipe_m11_wheel_command(&bipe_m11_axes[j], out.wheel_torque[j - 4], &frames[j]);
    }
}

static uint32_t dwt_cycles() { return ARM_DWT_CYCCNT; }

/* Serial: 'c' or 'l' chooses the controller (refused while torque is on), 'p' runs the
   bench parity check of the forward pass against the export's recorded vectors. */
static void serial_command() {
    if (!Serial.available()) return;
    const int c = Serial.read();
    if (c == 'c' || c == 'l') {
        const bool taken = bipe_selector_choose(
            &drive.selector, c == 'l' ? BIPE_CONTROLLER_LEARNED : BIPE_CONTROLLER_CLASSICAL,
            supervisor.state);
        Serial.printf("bipe m11: controller %s %s\n", c == 'l' ? "learned" : "classical",
                      taken ? "chosen" : "refused (torque on, unavailable or locked out)");
    } else if (c == 'p' && supervisor.state != BIPE_OFF && supervisor.state != BIPE_FAULT) {
        Serial.printf("bipe m11 parity: refused (runs only before start or after a fault: "
                      "16 passes overrun the cycle deadline)\n");
    } else if (c == 'p') {
        const bipe_parity_result r = bipe_locomotion_parity(bipe_locomotion_forward, dwt_cycles);
        const double us = 1e6 / F_CPU_ACTUAL;
        Serial.printf("bipe m11 parity: %s vectors=%u max_error=%.3g forward=%.0f-%.0fus "
                      "checkpoint=%s\n", r.passed ? "pass" : "FAIL", r.vectors, r.max_error,
                      r.min_cycles * us, r.max_cycles * us, bipe_locomotion_checkpoint());
    }
}

void setup() {
    pinMode(BIPE_M11_PIN_CAN_STBY, OUTPUT);
    digitalWrite(BIPE_M11_PIN_CAN_STBY, HIGH); /* MCP2562FD: STBY high = standby */
    pinMode(BIPE_M11_PIN_LED_DATA, OUTPUT);
    digitalWrite(BIPE_M11_PIN_LED_DATA, LOW);
    pinMode(BIPE_M11_PIN_IMU_CS, OUTPUT);
    digitalWrite(BIPE_M11_PIN_IMU_CS, HIGH);
    pinMode(BIPE_M11_PIN_IMU_INT1, INPUT);
    pinMode(BIPE_M11_PIN_ARM_N, INPUT_PULLUP);
    pinMode(BIPE_M11_PIN_STOP_N, INPUT_PULLUP);
    pinMode(BIPE_M11_PIN_PWR_GOOD_N, INPUT); /* 10k pull-up on the carrier */
    Serial.begin(115200);
    analogReadResolution(12);
    bipe_init(&supervisor);
    bipe_drive_init(&drive, bipe_locomotion_checkpoint());
    SPI.begin();
    if (imu_read(0x0f) == BIPE_M11_IMU_WHO_AM_I) {
        uint8_t reg[4], value[4];
        for (unsigned i = 0, n = bipe_lsm6dso_setup(reg, value); i < n; i++)
            imu_write(reg[i], value[i]);
        imu_ok = true;
    }
    can.begin();
    CANFD_timings_t timing;
    timing.clock = CLK_80MHz;
    timing.baudrate = BIPE_M11_CAN_NOMINAL_BIT_S;
    timing.baudrateFD = BIPE_M11_CAN_DATA_BIT_S;
    timing.propdelay = 190;
    timing.bus_length = 1;
    timing.sample = 66.6;
    can_ok = can.setBaudRate(timing);
    can.setRegions(64);
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
    last_cycle = micros();
}

void loop() {
    imu_poll();
    const uint32_t now_us = micros();
    if (now_us - last_cycle < CYCLE_US) return;
    last_cycle += CYCLE_US;
    const uint32_t now = millis();
    inputs.estop_closed = digitalReadFast(BIPE_M11_PIN_STOP_N) == LOW;
    inputs.power_good = digitalReadFast(BIPE_M11_PIN_PWR_GOOD_N) == LOW; /* LM5060 nPGD */
    inputs.bus_V = bipe_m11_bus_volts((unsigned)analogRead(BIPE_M11_PIN_BUS_ADC));
    inputs.bus_off = !can_ok;
    inputs.imu_present = imu_ok;
    inputs.imu_valid = imu_ok && inputs.imu_sequence != 0 && now_us - imu_us < CYCLE_US;
    inputs.imu_time_ms = now - (now_us - imu_us) / 1000;
    inputs.host_time_ms = now; /* the command source is this image's own controller */

    bipe_m11_outputs out = bipe_m11_outputs_for(&supervisor, BIPE_HARDWARE_QUALIFIED, inputs.power_good);
    digitalWrite(BIPE_M11_PIN_CAN_STBY, out.transceiver_on ? LOW : HIGH);
    bipe_fd_frame frames[BIPE_MOTORS];
    uint32_t t0 = ARM_DWT_CYCCNT;
    if (out.transceiver_on) feedback(now);
    uint32_t t1 = ARM_DWT_CYCCNT;
    compute(now, frames);
    uint32_t t2 = ARM_DWT_CYCCNT;

    const bool arm = digitalReadFast(BIPE_M11_PIN_ARM_N) == LOW;
    if (arm && !previous_arm) {
        if (supervisor.state == BIPE_OFF) bipe_start(&supervisor);
        else bipe_arm(&supervisor);
    }
    previous_arm = arm;
    const bipe_settings settings = bipe_default_settings();
    bipe_tick(&supervisor, &settings, &inputs, now, BIPE_HARDWARE_QUALIFIED);
    out = bipe_m11_outputs_for(&supervisor, BIPE_HARDWARE_QUALIFIED, inputs.power_good);
    if (out.transceiver_on) {
        bipe_fd_frame stop;
        for (unsigned j = 0; j < BIPE_MOTORS; j++) {
            /* out.commands is false while the profile is unqualified (static_assert). */
            if (out.commands) send(frames[j]);
            else if (bipe_m11_stop(&bipe_m11_axes[j], &stop)) send(stop);
        }
    }
    uint32_t t3 = ARM_DWT_CYCCNT;
    /* The policy's forward pass, off the sample-to-actuation path: its action is held
       for 20 ms and applies from the next tick. A non-finite action hands over to the
       classical controller (bipe_drive_forward). */
    if (bipe_drive_forward(&drive, bipe_locomotion_forward))
        worst_policy = max(worst_policy, ARM_DWT_CYCCNT - t3);
    serial_command();
    worst_feedback = max(worst_feedback, t1 - t0);
    worst_compute = max(worst_compute, t2 - t1);
    worst_commands = max(worst_commands, t3 - t2);
    if (now - last_report >= 1000 && Serial.availableForWrite() > 96) {
        last_report = now;
        const double us = 1e6 / F_CPU_ACTUAL;
        Serial.printf("bipe m11: state=%u fault=%u bus=%.2fV imu=%u can=%u feedback=%.0fus "
                      "compute=%.0fus commands=%.0fus policy=%.0fus missing=%lu "
                      "controller=%s handovers=%u clamps=%u/%u torque=locked\n",
                      (unsigned)supervisor.state, (unsigned)supervisor.fault, inputs.bus_V,
                      (unsigned)inputs.imu_valid, (unsigned)can_ok, worst_feedback * us,
                      worst_compute * us, worst_commands * us, worst_policy * us,
                      (unsigned long)missing,
                      drive.selector.running == BIPE_CONTROLLER_LEARNED ? "learned" : "classical",
                      drive.selector.handovers, drive.learned.leg_clamps,
                      drive.learned.wheel_clamps);
        worst_feedback = worst_compute = worst_commands = worst_policy = 0;
        missing = 0;
    }
}
