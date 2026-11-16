#ifndef BIPE_STARTUP_MONITOR_H
#define BIPE_STARTUP_MONITOR_H
#include <stdbool.h>
#include <stdint.h>
/* Portable independent MCU algorithm. mV already includes calibrated ADC gain;
   ±250 mV per channel, 5 ms sampling, ≤10 ms watchdog/cutoff are requirements. */
typedef struct {
    uint32_t now_ms, sequence, pack_mV, bus_mV;
    bool valid, armed;
} bipe_startup_sample;
typedef struct {
    uint32_t last_ms, last_sequence, cooldown_ms, started_ms, previous_delta_mV;
    uint64_t energy_upper_uJ;
    bool seen, low_seen, attempting, conducting, start_ok, fault;
} bipe_startup_monitor;
void bipe_startup_init(bipe_startup_monitor *m,uint32_t now);
bool bipe_startup_update(bipe_startup_monitor *m,const bipe_startup_sample *s);
void bipe_startup_tick(bipe_startup_monitor *m,uint32_t now);
#endif
