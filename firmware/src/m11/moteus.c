#include "m11_moteus.h"
#include <math.h>
#include <string.h>

typedef struct { bipe_fd_frame *f; bool ok; } writer;
/* int8, int16, int32 scales per register kind (moteus registers.md "Mappings"). */
typedef struct { double s[3]; } scale;
static const scale POSITION = {{0.01, 0.0001, 0.00001}}, VELOCITY = {{0.1, 0.00025, 0.00001}},
    ACCEL = {{0.05, 0.001, 0.00001}}, TORQUE = {{0.5, 0.01, 0.001}},
    PWM = {{1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0}}, VOLTAGE = {{0.5, 0.1, 0.001}},
    TEMPERATURE = {{1.0, 0.1, 0.001}}, TIME = {{0.01, 0.001, 0.000001}},
    CURRENT = {{1.0, 0.1, 0.001}}, POWER = {{10.0, 0.05, 0.0001}}, UNIT = {{1, 1, 1}};
/* 0x020-0x02d in order: position, velocity, feedforward torque, kp scale, kd scale,
   maximum torque, stop position, watchdog timeout, velocity limit, accel limit, fixed
   voltage override, ilimit scale, fixed current override, ignore position bounds. */
static const scale *const COMMAND[BIPE_MX_POSITION_REGISTERS] = {
    &POSITION, &VELOCITY, &TORQUE, &PWM, &PWM, &TORQUE, &POSITION, &TIME, &VELOCITY, &ACCEL,
    &VOLTAGE, &PWM, &CURRENT, &UNIT};
/* Replies 0x000-0x00f; NULL marks integer registers (mode, flags, fault) and 0x008/9,
   which this host never requests. */
static const scale *const REPLY[BIPE_MX_REPLY_REGISTERS] = {
    NULL, &POSITION, &VELOCITY, &TORQUE, &CURRENT, &CURRENT, &POSITION, &POWER, NULL, NULL,
    &TEMPERATURE, NULL, NULL, &VOLTAGE, &TEMPERATURE, NULL};
static const bool REPLY_KNOWN[BIPE_MX_REPLY_REGISTERS] = {
    1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1};
static const unsigned SIZE[4] = {1, 2, 4, 4};

static void put(writer *w, uint8_t byte) {
    if (w->f->size >= BIPE_FD_MAX) { w->ok = false; return; }
    w->f->data[w->f->size++] = byte;
}
static void put_le(writer *w, uint32_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; i++) put(w, (uint8_t)(value >> (8 * i)));
}
static void varuint(writer *w, unsigned value) {
    do {
        uint8_t byte = (uint8_t)(value & 0x7fu);
        value >>= 7;
        put(w, (uint8_t)(byte | (value ? 0x80u : 0u)));
    } while (value);
}
/* moteus multiplex saturate(): NaN/inf -> most negative integer; otherwise clamp the
   scaled value to +-max and truncate toward zero, as Python's int() does. */
static void put_mapped(writer *w, double value, unsigned resolution, const scale *k) {
    static const double MAX[3] = {127.0, 32767.0, 2147483647.0};
    if (resolution == BIPE_MX_F32) {
        float f = (float)value; uint32_t bits; memcpy(&bits, &f, sizeof bits);
        put_le(w, bits, 4); return;
    }
    int64_t raw;
    if (!isfinite(value)) raw = -(int64_t)MAX[resolution] - 1;
    else {
        double scaled = value / k->s[resolution];
        if (scaled < -MAX[resolution]) scaled = -MAX[resolution];
        else if (scaled > MAX[resolution]) scaled = MAX[resolution];
        raw = (int64_t)scaled;
    }
    put_le(w, (uint32_t)raw, SIZE[resolution]);
}
/* mjlib WriteCombiner: one subframe per run of equal resolution, counts <= 3 in the two
   LSBs of the subframe type. values == NULL writes the read (query) form. */
static void combine(writer *w, uint8_t base, unsigned start, const uint8_t *resolution,
                    unsigned n, const double *values, const scale *const *kinds) {
    int current = -1;
    for (unsigned i = 0; i < n; i++) {
        if (resolution[i] > BIPE_MX_IGNORE) { w->ok = false; return; }
        if ((int)resolution[i] != current) {
            current = resolution[i];
            if (current == BIPE_MX_IGNORE) continue;
            unsigned count = 1;
            while (i + count < n && resolution[i + count] == current) count++;
            uint8_t type = (uint8_t)(base | (unsigned)current << 2);
            if (count <= 3) put(w, (uint8_t)(type + count));
            else { put(w, type); put(w, (uint8_t)count); }
            varuint(w, start + i);
        }
        if (current != BIPE_MX_IGNORE && values) {
            /* Integer registers are truncated before mapping (the package's write_int). */
            double v = kinds[i] == &UNIT && isfinite(values[i]) ? trunc(values[i]) : values[i];
            put_mapped(w, v, (unsigned)current, kinds[i]);
        }
    }
}
static void query(writer *w, const bipe_mx_query *q) {
    combine(w, 0x10, BIPE_MX_MODE, q->resolution, 8, NULL, NULL);
    combine(w, 0x10, 0x00a, q->resolution + 8, 6, NULL, NULL);
}
static void begin(bipe_fd_frame *f, uint8_t destination, uint8_t source, bool reply) {
    memset(f, 0, sizeof *f);
    f->id = (uint32_t)(destination & 0x7fu) | (uint32_t)(source & 0x7fu) << 8 |
            (reply ? 0x8000u : 0u);
}
static bool finish(writer *w) { return w->ok && w->f->size && bipe_fd_pad(w->f); }

bool bipe_fd_pad(bipe_fd_frame *f) {
    static const uint8_t SIZES[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    if (!f || f->size > BIPE_FD_MAX) return false;
    for (unsigned i = 0; i < sizeof SIZES; i++)
        if (SIZES[i] >= f->size) {
            while (f->size < SIZES[i]) f->data[f->size++] = 0x50;
            return true;
        }
    return false;
}
bipe_mx_query bipe_mx_default_query(void) {
    bipe_mx_query q;
    memset(q.resolution, BIPE_MX_IGNORE, sizeof q.resolution);
    q.resolution[0] = BIPE_MX_INT8;
    q.resolution[1] = q.resolution[2] = q.resolution[3] = BIPE_MX_F32;
    q.resolution[8 + 3] = q.resolution[8 + 4] = q.resolution[8 + 5] = BIPE_MX_INT8;
    return q;
}
bipe_mx_position bipe_mx_empty_position(void) {
    bipe_mx_position c;
    memset(c.value, 0, sizeof c.value);
    memset(c.resolution, BIPE_MX_IGNORE, sizeof c.resolution);
    return c;
}
bool bipe_mx_position_frame(uint8_t destination, uint8_t source, const bipe_mx_position *c,
                            const bipe_mx_query *q, bipe_fd_frame *f) {
    if (!c || !f) return false;
    begin(f, destination, source, q != NULL);
    writer w = {f, true};
    put(&w, 0x01); put(&w, BIPE_MX_MODE); put(&w, BIPE_MX_POSITION);
    combine(&w, 0x00, 0x020, c->resolution, BIPE_MX_POSITION_REGISTERS, c->value, COMMAND);
    if (q) query(&w, q);
    return finish(&w);
}
bool bipe_mx_query_frame(uint8_t destination, uint8_t source, const bipe_mx_query *q,
                         bipe_fd_frame *f) {
    if (!q || !f) return false;
    begin(f, destination, source, true);
    writer w = {f, true};
    query(&w, q);
    return finish(&w);
}
bool bipe_mx_stop_frame(uint8_t destination, uint8_t source, const bipe_mx_query *q,
                        bipe_fd_frame *f) {
    if (!f) return false;
    begin(f, destination, source, q != NULL);
    writer w = {f, true};
    put(&w, 0x01); put(&w, BIPE_MX_MODE); put(&w, BIPE_MX_STOPPED);
    if (q) query(&w, q);
    return finish(&w);
}

typedef struct { const uint8_t *p; unsigned n, at; bool ok; } reader;
static uint8_t get(reader *r) {
    if (r->at >= r->n) { r->ok = false; return 0; }
    return r->p[r->at++];
}
static unsigned get_varuint(reader *r) {
    unsigned value = 0;
    for (unsigned shift = 0; shift < 35; shift += 7) {
        uint8_t byte = get(r);
        value |= (unsigned)(byte & 0x7fu) << shift;
        if (!(byte & 0x80u)) return value;
    }
    r->ok = false;
    return 0;
}
static double get_value(reader *r, unsigned resolution, const scale *k) {
    uint32_t raw = 0;
    for (unsigned i = 0; i < SIZE[resolution]; i++) raw |= (uint32_t)get(r) << (8 * i);
    if (resolution == BIPE_MX_F32) { float f; memcpy(&f, &raw, sizeof f); return f; }
    static const int64_t BIAS[3] = {256, 65536, 4294967296LL};
    static const uint32_t SIGN[3] = {0x80u, 0x8000u, 0x80000000u};
    int64_t v = (raw & SIGN[resolution]) ? (int64_t)raw - BIAS[resolution] : (int64_t)raw;
    if (!k) return (double)v;
    /* The most negative integer encodes NaN (moteus protocol _nanify). */
    if (v == -BIAS[resolution] / 2) return NAN;
    return (double)v * k->s[resolution];
}
bool bipe_mx_parse(const bipe_fd_frame *f, uint8_t host, bipe_mx_reply *out) {
    if (!f || !out || f->size > BIPE_FD_MAX || (f->id & 0x7fu) != (host & 0x7fu)) return false;
    memset(out, 0, sizeof *out);
    out->source = (uint8_t)((f->id >> 8) & 0x7fu);
    out->destination = (uint8_t)(f->id & 0x7fu);
    reader r = {f->data, f->size, 0, true};
    while (r.ok && r.at < r.n) {
        uint8_t type = get(&r);
        if (type == 0x50) continue; /* NOP padding */
        if ((type & 0xf0u) != 0x20u) return false;
        unsigned resolution = (type >> 2) & 3u, count = type & 3u;
        if (!count) count = get_varuint(&r);
        unsigned start = get_varuint(&r);
        if (!r.ok || !count || start + count > BIPE_MX_REPLY_REGISTERS) return false;
        for (unsigned i = start; i < start + count; i++) {
            if (!REPLY_KNOWN[i]) return false;
            out->value[i] = get_value(&r, resolution, REPLY[i]);
            out->present = (uint16_t)(out->present | 1u << i);
        }
    }
    return r.ok;
}
