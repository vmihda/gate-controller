#include "nice_sim.h"

#include "flash_timing.h"

#define PERMILLE_FULL 1000

static uint32_t elapsed(uint32_t now_ms, uint32_t since_ms) { return now_ms - since_ms; }

static int32_t travel_permille(const nice_sim_t *s, uint32_t duration_ms)
{
    const uint64_t moved = (uint64_t)duration_ms * PERMILLE_FULL / s->cfg.travel_ms;
    return moved > PERMILLE_FULL ? PERMILLE_FULL : (int32_t)moved;
}

static void start_motion(nice_sim_t *s, nice_sim_state_t state, uint32_t now_ms)
{
    s->state = state;
    s->base_permille = s->position_permille;
    s->motion_start_ms = now_ms;
}

static void stop(nice_sim_t *s, bool moved_open)
{
    s->state = NICE_SIM_STOPPED;
    s->last_move_open = moved_open;
}

/* Brings the position and the motion state up to now_ms. */
static void advance(nice_sim_t *s, uint32_t now_ms)
{
    const uint32_t moving_ms = elapsed(now_ms, s->motion_start_ms);
    switch (s->state) {
    case NICE_SIM_OPENING:
    case NICE_SIM_REVERSING: {
        const bool reversal_over = s->state == NICE_SIM_REVERSING && moving_ms >= NICE_SIM_REVERSE_MS;
        const int32_t position = s->base_permille +
                                 travel_permille(s, reversal_over ? NICE_SIM_REVERSE_MS : moving_ms);
        s->position_permille = position > PERMILLE_FULL ? PERMILLE_FULL : position;
        if (reversal_over || s->position_permille == PERMILLE_FULL) {
            stop(s, true);
        }
        break;
    }
    case NICE_SIM_CLOSING: {
        const int32_t position = s->base_permille - travel_permille(s, moving_ms);
        s->position_permille = position < 0 ? 0 : position;
        if (s->position_permille == 0) {
            s->state = NICE_SIM_CLOSED;
            s->last_move_open = false;
        }
        break;
    }
    case NICE_SIM_CLOSED:
    case NICE_SIM_STOPPED:
        break;
    }
}

/* Open - stop - close - stop: one step command. */
static void step(nice_sim_t *s, uint32_t now_ms)
{
    switch (s->state) {
    case NICE_SIM_CLOSED:
        start_motion(s, NICE_SIM_OPENING, now_ms);
        break;
    case NICE_SIM_OPENING:
        stop(s, true);
        break;
    case NICE_SIM_CLOSING:
        stop(s, false);
        break;
    case NICE_SIM_STOPPED:
        /* Fully open or stopped after opening: close. Stopped after closing: open. */
        start_motion(s, s->last_move_open ? NICE_SIM_CLOSING : NICE_SIM_OPENING, now_ms);
        break;
    case NICE_SIM_REVERSING:
        break;  /* the reversal runs to its end */
    }
}

/* The rule fires at 3000 ms (Nice says "longer than 3 s"; >= is the conservative reading for a safety oracle).
 * It is also checked on release, so a ticker slower than the hold cannot miss it. */
static void check_always_open(nice_sim_t *s, uint32_t now_ms)
{
    if (!s->pp_closed || s->always_open_done || elapsed(now_ms, s->pp_closed_ms) < NICE_SIM_ALWAYS_OPEN_MS) {
        return;
    }
    s->always_open_done = true;
    s->always_open_seen = true;
    if (s->state != NICE_SIM_OPENING && s->position_permille < PERMILLE_FULL) {
        start_motion(s, NICE_SIM_OPENING, now_ms);
    }
}

void nice_sim_init(nice_sim_t *s, const nice_sim_config_t *cfg, uint32_t now_ms)
{
    *s = (nice_sim_t){.cfg = *cfg, .state = NICE_SIM_CLOSED, .motion_start_ms = now_ms,
                     .pp_closed_ms = now_ms, .chop_epoch_ms = now_ms};
    if (s->cfg.travel_ms == 0u) {
        s->cfg.travel_ms = NICE_SIM_DEFAULT_TRAVEL_MS;
    }
}

void nice_sim_set_pp(nice_sim_t *s, bool closed, uint32_t now_ms)
{
    advance(s, now_ms);
    if (closed && !s->pp_closed) {
        s->pp_closed = true;
        s->pp_closed_ms = now_ms;
        s->always_open_done = false;
        step(s, now_ms);
    } else if (!closed && s->pp_closed) {
        check_always_open(s, now_ms);
        s->pp_closed = false;
        s->last_closure_ms = elapsed(now_ms, s->pp_closed_ms);
    }
}

void nice_sim_obstacle(nice_sim_t *s, uint32_t now_ms)
{
    advance(s, now_ms);
    if (s->state == NICE_SIM_CLOSING) {
        start_motion(s, NICE_SIM_REVERSING, now_ms);
    }
}

void nice_sim_tick(nice_sim_t *s, uint32_t now_ms)
{
    advance(s, now_ms);
    check_always_open(s, now_ms);
}

static bool blink_level(const nice_sim_t *s, uint32_t now_ms, uint32_t period_ms)
{
    const uint32_t phase_ms = elapsed(now_ms, s->motion_start_ms) % period_ms;
    return phase_ms * 100u < period_ms * FLASH_BLINK_DUTY_PCT;
}

/* Chop phase counts from the model start, so the 32-bit clock wrap does not shift it. */
static bool chop_level(const nice_sim_t *s, uint32_t now_ms)
{
    const uint32_t period_ms = 1000u / s->cfg.chop_hz > 0u ? 1000u / s->cfg.chop_hz : 1u;
    return (elapsed(now_ms, s->chop_epoch_ms) % period_ms) * 100u < period_ms * s->cfg.chop_duty_pct;
}

bool nice_sim_flash(const nice_sim_t *s, uint32_t now_ms)
{
    bool lit = false;
    switch (s->state) {
    case NICE_SIM_CLOSED:
        break;
    case NICE_SIM_STOPPED:
        lit = true;
        break;
    case NICE_SIM_OPENING:
    case NICE_SIM_REVERSING:
        lit = blink_level(s, now_ms, FLASH_SLOW_PERIOD_MS);
        break;
    case NICE_SIM_CLOSING:
        lit = blink_level(s, now_ms, FLASH_FAST_PERIOD_MS);
        break;
    }
    return lit && (s->cfg.chop_hz == 0u || chop_level(s, now_ms));
}

int32_t nice_sim_position_permille(const nice_sim_t *s) { return s->position_permille; }

nice_sim_state_t nice_sim_state(const nice_sim_t *s) { return s->state; }

uint32_t nice_sim_last_pp_closure_ms(const nice_sim_t *s) { return s->last_closure_ms; }

bool nice_sim_always_open_seen(const nice_sim_t *s) { return s->always_open_seen; }
