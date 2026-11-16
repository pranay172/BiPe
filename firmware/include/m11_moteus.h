#ifndef BIPE_M11_MOTEUS_H
#define BIPE_M11_MOTEUS_H
#include <stdbool.h>
#include <stdint.h>

/* mjbots moteus register protocol over CAN-FD (moteus docs/protocol/can.md and
   registers.md at commit 8f747f4). Frames are laid out the way the mjbots moteus Python
   package 1.1.1 lays them out (WriteCombiner grouping, saturation, truncation, NaN as the
   most negative integer); tests/test_m11_moteus.py checks the bytes against it. */
#define BIPE_FD_MAX 64
#define BIPE_MX_POSITION_REGISTERS 14 /* 0x020 position ... 0x02d ignore position bounds */
#define BIPE_MX_QUERY_REGISTERS 14    /* 0x000-0x007, then 0x00a-0x00f */
#define BIPE_MX_REPLY_REGISTERS 16    /* 0x000-0x00f */
enum bipe_mx_resolution { BIPE_MX_INT8, BIPE_MX_INT16, BIPE_MX_INT32, BIPE_MX_F32,
                          BIPE_MX_IGNORE };
enum bipe_mx_mode { BIPE_MX_STOPPED = 0, BIPE_MX_FAULT = 1, BIPE_MX_POSITION = 10,
                    BIPE_MX_TIMEOUT = 11, BIPE_MX_ZERO_VELOCITY = 12 };
enum bipe_mx_register { BIPE_MX_MODE = 0x000, BIPE_MX_POSITION_REG = 0x001,
    BIPE_MX_VELOCITY = 0x002, BIPE_MX_TORQUE = 0x003, BIPE_MX_Q_CURRENT = 0x004,
    BIPE_MX_VOLTAGE = 0x00d, BIPE_MX_TEMPERATURE = 0x00e, BIPE_MX_FAULT_CODE = 0x00f };
/* CAN-FD frame: 29-bit id (sent extended when above 0x7ff), bit-rate switched. */
typedef struct { uint32_t id; uint8_t size; uint8_t data[BIPE_FD_MAX]; } bipe_fd_frame;
/* Position-mode command values in output revolutions, Hz, N*m, scale and seconds, in
   register order from 0x020. A register with BIPE_MX_IGNORE resolution is not sent;
   NaN is sent as the protocol's "unset" value. */
typedef struct { double value[BIPE_MX_POSITION_REGISTERS];
                 uint8_t resolution[BIPE_MX_POSITION_REGISTERS]; } bipe_mx_position;
typedef struct { uint8_t resolution[BIPE_MX_QUERY_REGISTERS]; } bipe_mx_query;
typedef struct { uint8_t source, destination; uint16_t present;
                 double value[BIPE_MX_REPLY_REGISTERS]; } bipe_mx_reply;

/* The moteus Python package's default query: mode int8; position, velocity, torque f32;
   voltage, temperature, fault int8. */
bipe_mx_query bipe_mx_default_query(void);
/* All position registers ignored; set value and resolution for those to send. */
bipe_mx_position bipe_mx_empty_position(void);
/* query may be NULL (no reply requested). Returns false if the frame would not fit. */
bool bipe_mx_position_frame(uint8_t destination, uint8_t source, const bipe_mx_position *c,
                            const bipe_mx_query *query, bipe_fd_frame *frame);
bool bipe_mx_query_frame(uint8_t destination, uint8_t source, const bipe_mx_query *query,
                         bipe_fd_frame *frame);
bool bipe_mx_stop_frame(uint8_t destination, uint8_t source, const bipe_mx_query *query,
                        bipe_fd_frame *frame);
/* Pads to the next CAN-FD size with NOP (0x50). The builders call it. */
bool bipe_fd_pad(bipe_fd_frame *frame);
/* Decodes reply subframes for registers 0x000-0x00f into SI-like moteus units. Rejects
   write/read error subframes, unknown subframes, registers outside the table, truncated
   values and frames whose destination is not `host`. */
bool bipe_mx_parse(const bipe_fd_frame *frame, uint8_t host, bipe_mx_reply *reply);
#endif
