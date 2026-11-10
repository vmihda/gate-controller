#include "gate_controller.h"

#include <stddef.h>

#include "gate_config.h"

/* The longest plan (CLOSE after a stopped closing) needs three pulses; OPEN needs at most two
 * plus the reserved stop. A longer contact closure risks the Nice "always open" mode (> 3 s). */
_Static_assert(GATE_MAX_PULSES >= 3u, "the command plans need three pulses");
_Static_assert(GATE_PULSE_MS < 1500u, "the spec allows at most 1.5 s of relay contact");

#define PULSE_SPACING_MS (GATE_PULSE_SPACING_MS + GATE_PULSE_SPACING_MARGIN_MS)
#define CONFIRM_LIMIT_MS (GATE_PULSE_MS + GATE_CONFIRM_TIMEOUT_MS)  /* counted from the pulse start */
#define DUE_LIMIT_MS GATE_CONFIRM_TIMEOUT_MS  /* longest wait of a planned pulse, counted from the plan */

static uint32_t elapsed(uint32_t now_ms, uint32_t then_ms)
{
    return (uint32_t)(now_ms - then_ms);
}

static void finish(gate_controller_t *c, gate_error_t error)
{
    c->active = false;
    c->error = error;
}

static void set_phase(gate_controller_t *c, gate_phase_t phase, uint32_t now_ms)
{
    c->phase = phase;
    c->phase_ms = now_ms;
}

static void plan_pulse(gate_controller_t *c, gate_pulse_kind_t kind, gate_state_t from, uint32_t now_ms)
{
    c->pulse_kind = kind;
    c->pulse_from = from;
    set_phase(c, GATE_PHASE_PULSE, now_ms);
}

static void start_travel(gate_controller_t *c, uint32_t now_ms)
{
    set_phase(c, GATE_PHASE_TRAVEL, now_ms);
}

static gate_state_t travel_state(gate_cmd_t cmd)
{
    return cmd == GATE_CMD_OPEN ? GATE_STATE_OPENING : GATE_STATE_CLOSING;
}

/* Clocks that outlive a command. Each flag is dropped once its window has passed, so a long idle
 * time can never wrap the subtraction into a false "recent". */
static void track(gate_controller_t *c, gate_state_t state, uint32_t now_ms)
{
    const uint32_t since_pulse = elapsed(now_ms, c->pulse_ms);
    if (c->pulse_recent && since_pulse >= PULSE_SPACING_MS) {
        c->pulse_recent = false;
    }
    if (c->stale && (state != c->stale_state || since_pulse >= CONFIRM_LIMIT_MS)) {
        c->stale = false;
    }
    if (state != GATE_STATE_UNKNOWN) {
        c->unknown_timing = false;
    } else if (!c->unknown_timing) {
        c->unknown_timing = true;
        c->unknown_since_ms = now_ms;
    }
    if (state != c->seen_state) {
        c->seen_state = state;
        c->seen_since_ms = now_ms;
        c->seen_confirmed = false;
    } else if (!c->seen_confirmed && elapsed(now_ms, c->seen_since_ms) > GATE_STEADY_AFTER_MS) {
        c->seen_confirmed = true;
    }
}

/* See gate_controller_step in the header: a motion must be fresh and confirmed before a pulse may stop it.
 * A pulse from a steady state (CLOSED, NOT_CLOSED) needs no edge. */
static bool evidence_holds(const gate_controller_t *c, gate_state_t state, bool motion_fresh)
{
    const bool moving = state == GATE_STATE_OPENING || state == GATE_STATE_CLOSING;
    return !moving || (c->seen_confirmed && motion_fresh);
}

static bool unknown_too_long(const gate_controller_t *c, uint32_t now_ms)
{
    return c->unknown_timing && elapsed(now_ms, c->unknown_since_ms) >= GATE_UNKNOWN_WAIT_MS;
}

/* The table of the spec for the first recognized state. A gate already moving toward the target gets no
 * pulse, but the command stays active through the travel, so an obstacle or a stop on the way is still
 * reported (final review I-1). Called only with a recognized state. */
static void start_plan(gate_controller_t *c, gate_state_t state, gate_motion_t last_motion, uint32_t now_ms)
{
    if (c->cmd == GATE_CMD_OPEN) {
        switch (state) {
        case GATE_STATE_CLOSED: plan_pulse(c, GATE_KIND_OPEN_GO, state, now_ms); break;
        case GATE_STATE_CLOSING: plan_pulse(c, GATE_KIND_OPEN_STOP, state, now_ms); break;
        case GATE_STATE_OPENING: start_travel(c, now_ms); break;
        case GATE_STATE_NOT_CLOSED: finish(c, GATE_ERR_NONE); break;  /* halfway counts as open */
        case GATE_STATE_UNKNOWN: break;                               /* not reached */
        }
        return;
    }
    switch (state) {
    case GATE_STATE_NOT_CLOSED:
        plan_pulse(c, last_motion == GATE_MOTION_CLOSING ? GATE_KIND_CLOSE_REVERSE : GATE_KIND_CLOSE_GO, state, now_ms);
        break;
    case GATE_STATE_OPENING: plan_pulse(c, GATE_KIND_CLOSE_STOP, state, now_ms); break;
    case GATE_STATE_CLOSING: start_travel(c, now_ms); break;
    case GATE_STATE_CLOSED: finish(c, GATE_ERR_NONE); break;
    case GATE_STATE_UNKNOWN: break;  /* not reached */
    }
}

/* A recognized state the current phase did not plan for. */
static void on_unplanned(gate_controller_t *c, gate_state_t state, uint32_t now_ms)
{
    if (c->cmd == GATE_CMD_CLOSE) {
        switch (state) {
        case GATE_STATE_CLOSED: finish(c, GATE_ERR_NONE); break;
        case GATE_STATE_CLOSING: start_travel(c, now_ms); break;
        case GATE_STATE_OPENING: finish(c, GATE_ERR_OBSTACLE); break;  /* overrides everything */
        default: finish(c, GATE_ERR_UNEXPECTED_STOP); break;
        }
        return;
    }
    switch (state) {
    case GATE_STATE_OPENING: start_travel(c, now_ms); break;
    case GATE_STATE_CLOSING: plan_pulse(c, GATE_KIND_WRONG_STOP, state, now_ms); break;
    default: finish(c, GATE_ERR_UNEXPECTED_STOP); break;
    }
}

static gate_state_t expected_after(gate_pulse_kind_t kind)
{
    switch (kind) {
    case GATE_KIND_OPEN_GO:
    case GATE_KIND_CLOSE_REVERSE: return GATE_STATE_OPENING;
    case GATE_KIND_CLOSE_GO: return GATE_STATE_CLOSING;
    case GATE_KIND_OPEN_STOP:
    case GATE_KIND_CLOSE_STOP:
    case GATE_KIND_WRONG_STOP: break;
    }
    return GATE_STATE_NOT_CLOSED;
}

/* The expected reaction came: plan the next pulse or start the final travel. */
static void advance(gate_controller_t *c, gate_state_t state, uint32_t now_ms)
{
    switch (c->pulse_kind) {
    case GATE_KIND_OPEN_STOP: plan_pulse(c, GATE_KIND_OPEN_GO, state, now_ms); break;
    case GATE_KIND_CLOSE_REVERSE: plan_pulse(c, GATE_KIND_CLOSE_STOP, state, now_ms); break;
    case GATE_KIND_CLOSE_STOP: plan_pulse(c, GATE_KIND_CLOSE_GO, state, now_ms); break;
    case GATE_KIND_OPEN_GO:
    case GATE_KIND_CLOSE_GO: start_travel(c, now_ms); break;
    case GATE_KIND_WRONG_STOP: break;  /* finishes at the pulse, never awaits */
    }
}

static bool try_fire(gate_controller_t *c, gate_state_t state, uint32_t now_ms)
{
    if (c->pulse_recent) {
        return false;
    }
    if (c->pulses >= GATE_MAX_PULSES) {
        /* Unreachable with the plans above; never exceed the budget if they change. */
        finish(c, GATE_ERR_NO_CONFIRM);
        return false;
    }
    if (c->pulse == NULL || !c->pulse(c->pulse_ctx)) {
        return false;  /* refused, nothing started: still due, asked again on the next step */
    }
    c->pulses++;
    c->pulse_recent = true;
    c->pulse_ms = now_ms;
    c->stale = true;
    c->stale_state = state;
    if (c->pulse_kind == GATE_KIND_WRONG_STOP) {
        finish(c, GATE_ERR_WRONG_DIRECTION);
    } else {
        set_phase(c, GATE_PHASE_AWAIT, now_ms);
    }
    return true;
}

void gate_controller_init(gate_controller_t *c, uint32_t now_ms, gate_pulse_fn pulse, void *pulse_ctx)
{
    (void)now_ms;  /* every clock starts on its first use */
    *c = (gate_controller_t){.active = false, .pulse = pulse, .pulse_ctx = pulse_ctx};
}

void gate_controller_command(gate_controller_t *c, gate_cmd_t cmd, uint32_t now_ms)
{
    if (c->active && c->cmd == cmd) {
        return;  /* the same command again (a double tap): the running plan goes on */
    }
    c->active = true;
    c->cmd = cmd;
    c->pulses = 0;
    c->error = GATE_ERR_NONE;
    set_phase(c, GATE_PHASE_START, now_ms);
    /* The wait for a recognized state counts from the command. */
    c->unknown_timing = true;
    c->unknown_since_ms = now_ms;
}

bool gate_controller_step(gate_controller_t *c, gate_state_t state, gate_motion_t last_motion, bool motion_fresh,
                          uint32_t now_ms)
{
    track(c, state, now_ms);
    if (!c->active) {
        return false;
    }
    /* Right after a pulse the decoder still reports the old state for a while; that reading says
     * nothing about the gate yet. UNKNOWN right after a pulse is normal for the same reason. */
    const bool unsure = state == GATE_STATE_UNKNOWN || (c->stale && state == c->stale_state);

    switch (c->phase) {
    case GATE_PHASE_START:
        if (!unsure) {
            start_plan(c, state, last_motion, now_ms);
        } else if (unknown_too_long(c, now_ms)) {
            finish(c, GATE_ERR_UNKNOWN_STATE);
        }
        break;
    case GATE_PHASE_PULSE:
        if (c->pulse_kind == GATE_KIND_WRONG_STOP) {
            /* Ruling R: the stop stays due while the gate may still close against OPEN, so neither a relay
             * that keeps refusing nor UNKNOWN drops it; the travel limit is the last bound. */
            if (elapsed(now_ms, c->phase_ms) >= GATE_MAX_TRAVEL_MS) {
                finish(c, GATE_ERR_TRAVEL_TIMEOUT);
            } else if (state != GATE_STATE_UNKNOWN && state != c->pulse_from) {
                finish(c, GATE_ERR_WRONG_DIRECTION);  /* no longer closing: nothing to stop */
            }
        } else if (elapsed(now_ms, c->phase_ms) >= DUE_LIMIT_MS) {
            /* Bounds every other wait for a due pulse, also a motion reading that keeps breaking into UNKNOWN. */
            finish(c, GATE_ERR_NO_CONFIRM);
        } else if (state == GATE_STATE_UNKNOWN) {
            if (unknown_too_long(c, now_ms)) {
                finish(c, GATE_ERR_UNKNOWN_STATE);
            }
        } else if (state != c->pulse_from) {
            on_unplanned(c, state, now_ms);
        }
        break;
    case GATE_PHASE_AWAIT:
        if (state == expected_after(c->pulse_kind)) {
            advance(c, state, now_ms);
        } else if (elapsed(now_ms, c->phase_ms) >= CONFIRM_LIMIT_MS) {
            finish(c, GATE_ERR_NO_CONFIRM);
        } else if (!unsure) {
            on_unplanned(c, state, now_ms);
        }
        break;
    case GATE_PHASE_TRAVEL:
        if (state == travel_state(c->cmd) || state == GATE_STATE_UNKNOWN) {
            if (unknown_too_long(c, now_ms)) {
                finish(c, GATE_ERR_UNKNOWN_STATE);
            } else if (elapsed(now_ms, c->phase_ms) >= GATE_MAX_TRAVEL_MS) {
                finish(c, GATE_ERR_TRAVEL_TIMEOUT);
            }
        } else if (c->cmd == GATE_CMD_OPEN && state == GATE_STATE_NOT_CLOSED) {
            finish(c, GATE_ERR_NONE);  /* halfway counts as open: FLASH cannot tell */
        } else {
            on_unplanned(c, state, now_ms);
        }
        break;
    }

    /* A due pulse fires only while the gate is in the planned state, so never into UNKNOWN. */
    return c->active && c->phase == GATE_PHASE_PULSE && state == c->pulse_from && evidence_holds(c, state, motion_fresh) &&
           try_fire(c, state, now_ms);
}

bool gate_controller_busy(const gate_controller_t *c)
{
    return c->active;
}

gate_error_t gate_controller_last_error(const gate_controller_t *c)
{
    return c->error;
}
