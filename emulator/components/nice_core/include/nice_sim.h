/* Model of the Nice SNA2 control unit in "open - stop - close - stop" step-by-step mode, with the FLASH
 * "open-gate indicator" output. Pure C11, no heap, no globals, no ESP-IDF dependency: the same code runs
 * in host tests, in the ESP32 emulator and in the Wokwi custom chip.
 * Times are uint32_t milliseconds and may wrap; they are only compared by subtraction.
 * Call nice_sim_tick often enough (every 1 ms or 10 ms) for the model to follow time. */
#ifndef NICE_SIM_H
#define NICE_SIM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NICE_SIM_DEFAULT_TRAVEL_MS 15000u
/* Contact held 3000 ms or longer means "always open". Nice documents "longer than 3 s"; this oracle fires at
 * the 3000 ms mark, the conservative side for checking the hardware limit. */
#define NICE_SIM_ALWAYS_OPEN_MS 3000u
#define NICE_SIM_REVERSE_MS 1000u       /* obstacle reversal: open for this long, then stop */

typedef struct {
    uint32_t travel_ms;     /* full travel time, closed to open */
    uint16_t chop_hz;       /* 0 = steady; else every "high" phase of FLASH is chopped at this rate;
                               the period is 1000 / chop_hz ms, rounded down to whole ms */
    uint8_t chop_duty_pct;  /* share of each chop period that is high */
} nice_sim_config_t;

typedef enum {
    NICE_SIM_CLOSED,
    NICE_SIM_OPENING,
    NICE_SIM_CLOSING,
    NICE_SIM_STOPPED,    /* stopped between the limits, or fully open */
    NICE_SIM_REVERSING   /* obstacle reversal: opening for NICE_SIM_REVERSE_MS */
} nice_sim_state_t;

/* Fields are private; the struct is public only so the caller can own it without a heap. */
typedef struct nice_sim {
    nice_sim_config_t cfg;
    nice_sim_state_t state;
    bool last_move_open;         /* direction of the last motion: decides what a step does when stopped */
    int32_t base_permille;       /* position at motion_start_ms */
    int32_t position_permille;   /* position at the last update */
    uint32_t motion_start_ms;
    bool pp_closed;
    bool always_open_done;       /* the 3 s mark of the current closure was already handled */
    bool always_open_seen;       /* sticky: "always open" was triggered at least once */
    uint32_t pp_closed_ms;
    uint32_t chop_epoch_ms;      /* chop phase origin: the init time */
    uint32_t last_closure_ms;
} nice_sim_t;

/* Starts closed. A zero cfg->travel_ms means NICE_SIM_DEFAULT_TRAVEL_MS. */
void nice_sim_init(nice_sim_t *s, const nice_sim_config_t *cfg, uint32_t now_ms);
/* Contact level of the Step-by-step input, from the relay or the wall button. A step fires on the closing edge. */
void nice_sim_set_pp(nice_sim_t *s, bool closed, uint32_t now_ms);
/* Obstacle input: only acts while closing. */
void nice_sim_obstacle(nice_sim_t *s, uint32_t now_ms);
void nice_sim_tick(nice_sim_t *s, uint32_t now_ms);
/* Instantaneous FLASH level, including chopping. */
bool nice_sim_flash(const nice_sim_t *s, uint32_t now_ms);
/* 0 closed, 1000 open; as of the last tick or input call. */
int32_t nice_sim_position_permille(const nice_sim_t *s);
nice_sim_state_t nice_sim_state(const nice_sim_t *s);
/* Measured length of the last completed contact closure. */
uint32_t nice_sim_last_pp_closure_ms(const nice_sim_t *s);
/* True once the "always open" rule fired; the bench must never trigger it. */
bool nice_sim_always_open_seen(const nice_sim_t *s);

#ifdef __cplusplus
}
#endif

#endif /* NICE_SIM_H */
