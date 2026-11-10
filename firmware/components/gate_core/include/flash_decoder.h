/* Decoder of the Nice FLASH "open-gate indicator" signal. Pure C, no heap, no ESP-IDF dependency.
 * Steady on = not closed, off = closed, slow blink = opening, fast blink = closing.
 * Times are uint32_t milliseconds and may wrap; they are only compared by subtraction. */
#ifndef FLASH_DECODER_H
#define FLASH_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GATE_STATE_UNKNOWN,
    GATE_STATE_CLOSED,
    GATE_STATE_NOT_CLOSED,
    GATE_STATE_OPENING,
    GATE_STATE_CLOSING
} gate_state_t;

typedef enum { GATE_MOTION_NONE, GATE_MOTION_OPENING, GATE_MOTION_CLOSING } gate_motion_t;

typedef struct flash_decoder {
    gate_state_t state;
    gate_motion_t last_motion;
    gate_motion_t run_motion;  /* motion recognized during the current blinking run */
    bool level_known;
    bool level_lit;     /* filtered level */
    bool pending_low;   /* raw falling edge not yet confirmed as longer than the glitch limit */
    bool run_active;    /* edges seen since the last steady state */
    bool have_edge[2];  /* indexed by edge direction: 0 falling, 1 rising */
    uint32_t last_edge_ms[2];
    uint32_t last_any_edge_ms;
    uint32_t fall_raw_ms;
} flash_decoder_t;

/* After init the caller must seed the current pin level with one flash_decoder_on_level call;
 * without it the state stays UNKNOWN. */
void flash_decoder_init(flash_decoder_t *d, uint32_t now_ms);
/* Feed every raw level change, including chopping. */
void flash_decoder_on_level(flash_decoder_t *d, bool lit, uint32_t now_ms);
/* Call every 10 ms. */
void flash_decoder_tick(flash_decoder_t *d, uint32_t now_ms);
gate_state_t flash_decoder_state(const flash_decoder_t *d);
/* True while the state is OPENING or CLOSING and the last decoded edge is at most
 * GATE_FRESH_EDGE_MS of that blink old: the gate is still blinking, not stopped under a held reading.
 * Call after flash_decoder_tick for now_ms. */
bool flash_decoder_motion_fresh(const flash_decoder_t *d, uint32_t now_ms);
/* Last motion before the current stationary state. */
gate_motion_t flash_decoder_last_motion(const flash_decoder_t *d);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_DECODER_H */
