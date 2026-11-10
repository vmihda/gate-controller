/* Drives OPEN and CLOSE through the single Step-by-step input of the Nice controller
 * ("open - stop - close - stop" mode) and checks every pulse against the gate state decoded from FLASH.
 * Pure C, no heap, no ESP-IDF dependency. Times are uint32_t milliseconds and may wrap; they are only
 * compared by subtraction. */
#ifndef GATE_CONTROLLER_H
#define GATE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "flash_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { GATE_CMD_OPEN, GATE_CMD_CLOSE } gate_cmd_t;

typedef enum {
    GATE_ERR_NONE,
    GATE_ERR_UNKNOWN_STATE,     /* the state was not recognized in time */
    GATE_ERR_NO_CONFIRM,        /* the expected reaction to a pulse did not come */
    GATE_ERR_WRONG_DIRECTION,   /* OPEN made the gate close; it was stopped once */
    GATE_ERR_OBSTACLE,          /* CLOSE saw an opening it did not ask for: Nice safety reversal */
    GATE_ERR_UNEXPECTED_STOP,   /* the gate stopped short of the target without our pulse */
    GATE_ERR_TRAVEL_TIMEOUT     /* the motion lasted longer than GATE_MAX_TRAVEL_MS */
} gate_error_t;

/* Starts one GATE_PULSE_MS relay pulse. True if the relay accepted it; false if it refused (then nothing
 * was started, and the controller asks again on a later step). */
typedef bool (*gate_pulse_fn)(void *ctx);

/* Private to gate_controller.c; public only so the caller can own the struct without a heap. */
typedef enum {
    GATE_PHASE_START,   /* the first recognized state picks the plan */
    GATE_PHASE_PULSE,   /* a pulse is due once the gap allows and the gate is still in pulse_from */
    GATE_PHASE_AWAIT,   /* waiting for the reaction to the last pulse */
    GATE_PHASE_TRAVEL   /* moving toward the target, no more pulses */
} gate_phase_t;

typedef enum {
    GATE_KIND_OPEN_GO,        /* from CLOSED or a stopped gate: expect OPENING */
    GATE_KIND_OPEN_STOP,      /* stop a closing gate: expect NOT_CLOSED, then GATE_KIND_OPEN_GO */
    GATE_KIND_CLOSE_GO,       /* from NOT_CLOSED: expect CLOSING */
    GATE_KIND_CLOSE_REVERSE,  /* stopped after closing, the pulse opens: expect OPENING, then GATE_KIND_CLOSE_STOP */
    GATE_KIND_CLOSE_STOP,     /* stop an opening gate: expect NOT_CLOSED, then GATE_KIND_CLOSE_GO */
    GATE_KIND_WRONG_STOP      /* OPEN saw closing: stop it and fail */
} gate_pulse_kind_t;

typedef struct gate_controller {
    bool active;
    gate_cmd_t cmd;
    gate_phase_t phase;
    gate_pulse_kind_t pulse_kind;
    gate_state_t pulse_from;  /* state required to fire the pending pulse */
    uint8_t pulses;           /* used by the active command */
    uint32_t phase_ms;        /* start of the phase; in GATE_PHASE_AWAIT the start of the pulse */
    gate_error_t error;
    gate_pulse_fn pulse;      /* from gate_controller_init */
    void *pulse_ctx;
    /* Kept across commands. */
    bool pulse_recent;        /* the last pulse is closer than the pulse spacing (see gate_controller.c) */
    uint32_t pulse_ms;        /* start of the last pulse */
    bool stale;               /* the decoder still shows the state we pulsed from */
    gate_state_t stale_state;
    bool unknown_timing;      /* UNKNOWN is being timed from unknown_since_ms */
    uint32_t unknown_since_ms;
    gate_state_t seen_state;  /* the decoder state of the last step, seen without a break since seen_since_ms */
    uint32_t seen_since_ms;
    bool seen_confirmed;      /* seen_state has lasted longer than GATE_STEADY_AFTER_MS */
} gate_controller_t;

/* `pulse` starts the relay pulses; it is called only from gate_controller_step. NULL refuses every pulse. */
void gate_controller_init(gate_controller_t *c, uint32_t now_ms, gate_pulse_fn pulse, void *pulse_ctx);
/* Starts a command; replaces an active one. The same command as the active one changes nothing, so a
 * double tap keeps the running plan. The pulse gap from the previous pulse is still kept. */
void gate_controller_command(gate_controller_t *c, gate_cmd_t cmd, uint32_t now_ms);
/* Call every 10 ms with the decoder's current state, after the decoder's tick at the same time.
 * When a pulse is due, the step starts it through `pulse` and returns true if the relay accepted it.
 * A refused pulse stays due: it uses no budget, the plan does not move on, and the next step asks again.
 * A due pulse that cannot be started within GATE_CONFIRM_TIMEOUT_MS of being planned (relay refusing,
 * gate never in the planned state, motion never confirmed) ends the command with an error. The one
 * exception is the stop for a wrong direction: it stays due while the gate may still be closing against
 * OPEN, and the command ends only when it is sent (WRONG_DIRECTION), when the gate is seen stopped or
 * closed (WRONG_DIRECTION, no pulse), or GATE_MAX_TRAVEL_MS after the plan (TRAVEL_TIMEOUT).
 * A pulse that stops a moving gate needs fresh, confirmed motion. Fresh: `motion_fresh`, from
 * flash_decoder_motion_fresh, says a FLASH edge came within one blink half (Ruling S); a stopped gate is
 * lit without edges, so a held reading of it is never fresh. Confirmed: OPENING or CLOSING reported
 * without a break for longer than GATE_STEADY_AFTER_MS. The decoder holds one reading at most that long after its
 * last edge, so a confirmed motion went on after it was recognized. A single reading can come from a
 * gate that already stopped (an obstacle reversal that ended, a period cut by a stop), and a pulse into
 * a stopped gate starts it. Until then the pulse waits; if the motion ends first, the command ends
 * with an error and no pulse. */
bool gate_controller_step(gate_controller_t *c, gate_state_t state, gate_motion_t last_motion, bool motion_fresh,
                          uint32_t now_ms);
bool gate_controller_busy(const gate_controller_t *c);
/* Result of the last finished command; cleared when a new command starts. */
gate_error_t gate_controller_last_error(const gate_controller_t *c);

#ifdef __cplusplus
}
#endif

#endif /* GATE_CONTROLLER_H */
