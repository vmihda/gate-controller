#include "flash_decoder.h"

#include "gate_config.h"

#define LIT_SHARE(period_ms) ((period_ms) * FLASH_BLINK_DUTY_PCT / 100u)
#define DARK_SHARE(period_ms) ((period_ms) * (100u - FLASH_BLINK_DUTY_PCT) / 100u)
#define HALF_JITTER_MS (2u * GATE_EDGE_JITTER_MS)

/* The widened slow and fast bands must stay apart, for the period and for each half. */
_Static_assert(GATE_PERIOD_MAX_MS(FLASH_FAST_PERIOD_MS) + GATE_EDGE_JITTER_MS <
                   GATE_PERIOD_MIN_MS(FLASH_SLOW_PERIOD_MS) - GATE_EDGE_JITTER_MS,
               "slow and fast period bands overlap");
_Static_assert(GATE_PERIOD_MAX_MS(LIT_SHARE(FLASH_FAST_PERIOD_MS)) + HALF_JITTER_MS <
                   GATE_PERIOD_MIN_MS(LIT_SHARE(FLASH_SLOW_PERIOD_MS)) - HALF_JITTER_MS,
               "slow and fast lit halves overlap");
_Static_assert(GATE_PERIOD_MAX_MS(DARK_SHARE(FLASH_FAST_PERIOD_MS)) + HALF_JITTER_MS <
                   GATE_PERIOD_MIN_MS(DARK_SHARE(FLASH_SLOW_PERIOD_MS)) - HALF_JITTER_MS,
               "slow and fast dark halves overlap");
_Static_assert(GATE_PERIOD_MIN_MS(LIT_SHARE(FLASH_FAST_PERIOD_MS)) > HALF_JITTER_MS &&
                   GATE_PERIOD_MIN_MS(DARK_SHARE(FLASH_FAST_PERIOD_MS)) > HALF_JITTER_MS,
               "the widened bands must not reach zero");

/* Freshness must be stricter than the hold of a reading, or it would add nothing. */
_Static_assert(GATE_FRESH_EDGE_MS(FLASH_SLOW_PERIOD_MS) < GATE_STEADY_AFTER_MS &&
                   GATE_FRESH_EDGE_MS(FLASH_FAST_PERIOD_MS) < GATE_STEADY_AFTER_MS,
               "a fresh edge must be younger than a held reading");

static bool fits(uint32_t ms, uint32_t nominal_ms, uint32_t jitter_ms)
{
    return ms >= GATE_PERIOD_MIN_MS(nominal_ms) - jitter_ms && ms <= GATE_PERIOD_MAX_MS(nominal_ms) + jitter_ms;
}

/* One period is a blink of the given nominal period only if the period and both its halves fit. A period
 * cut short by a stop has a full half of one blink and a truncated half that does not fit it (review C1:
 * a slow lit half and a slow dark half cut short by a stop make a period in the fast band).
 * One exception, for opening: FLASH stays lit through a stop and into the next motion, so a lit half can
 * hold the end of one motion and the start of the next, as when an obstacle reversal starts in a lit
 * phase of closing. A dark half is never made longer by an event (arriving closed only lets an opening
 * follow), so a full slow dark half after that lit half proves opening, and the lit half may be longer
 * than its band. A fast period gets no such exception: a stop can cut a slow dark half down to a fast one. */
static bool is_blink(uint32_t lit_ms, uint32_t dark_ms, uint32_t nominal_ms, bool long_lit_allowed)
{
    const bool lit_fits = long_lit_allowed
                              ? lit_ms >= GATE_PERIOD_MIN_MS(LIT_SHARE(nominal_ms)) - HALF_JITTER_MS
                              : fits(lit_ms, LIT_SHARE(nominal_ms), HALF_JITTER_MS);
    return fits(lit_ms + dark_ms, nominal_ms, GATE_EDGE_JITTER_MS) && lit_fits &&
           fits(dark_ms, DARK_SHARE(nominal_ms), HALF_JITTER_MS);
}

/* lit_first: the period ends on a rise, so its dark half is the later one. */
static gate_state_t classify(uint32_t lit_ms, uint32_t dark_ms, bool lit_first)
{
    if (is_blink(lit_ms, dark_ms, FLASH_SLOW_PERIOD_MS, lit_first)) {
        return GATE_STATE_OPENING;
    }
    if (is_blink(lit_ms, dark_ms, FLASH_FAST_PERIOD_MS, false)) {
        return GATE_STATE_CLOSING;
    }
    return GATE_STATE_UNKNOWN;
}

/* A filtered edge closes one period: from the previous edge in the same direction, through the opposite
 * edge between them (edges alternate), to this one. */
static void commit_edge(flash_decoder_t *d, bool lit, uint32_t at_ms)
{
    const unsigned dir = lit ? 1u : 0u;
    if (!d->run_active) {
        d->have_edge[0] = false;
        d->have_edge[1] = false;
        d->state = GATE_STATE_UNKNOWN;
        d->run_motion = GATE_MOTION_NONE;
        d->run_active = true;
    }
    d->level_lit = lit;
    if (d->have_edge[dir]) {
        const uint32_t middle_ms = d->last_edge_ms[1u - dir];
        const uint32_t first_half_ms = (uint32_t)(middle_ms - d->last_edge_ms[dir]);
        const uint32_t second_half_ms = (uint32_t)(at_ms - middle_ms);
        /* Ending on a rise, the period was lit, then dark; ending on a fall, dark, then lit. */
        d->state = lit ? classify(first_half_ms, second_half_ms, true) : classify(second_half_ms, first_half_ms, false);
        if (d->state == GATE_STATE_OPENING) {
            d->run_motion = GATE_MOTION_OPENING;
        } else if (d->state == GATE_STATE_CLOSING) {
            d->run_motion = GATE_MOTION_CLOSING;
        }
    }
    d->have_edge[dir] = true;
    d->last_edge_ms[dir] = at_ms;
    d->last_any_edge_ms = at_ms;
}

static void enter_steady(flash_decoder_t *d)
{
    /* The last period of a run may be truncated (UNKNOWN), so use the motion seen during the run. */
    if (d->run_motion != GATE_MOTION_NONE) {
        d->last_motion = d->run_motion;
    }
    d->state = d->level_lit ? GATE_STATE_NOT_CLOSED : GATE_STATE_CLOSED;
    d->run_active = false;
}

static void settle(flash_decoder_t *d, uint32_t now_ms)
{
    if (d->pending_low && (uint32_t)(now_ms - d->fall_raw_ms) > FLASH_GLITCH_MS) {
        d->pending_low = false;
        commit_edge(d, false, d->fall_raw_ms);
    }
    if (d->run_active && d->level_known && (uint32_t)(now_ms - d->last_any_edge_ms) > GATE_STEADY_AFTER_MS) {
        enter_steady(d);
    }
}

void flash_decoder_init(flash_decoder_t *d, uint32_t now_ms)
{
    *d = (flash_decoder_t){
        .state = GATE_STATE_UNKNOWN,
        .last_motion = GATE_MOTION_NONE,
        .run_active = true,
        .last_any_edge_ms = now_ms,
    };
}

void flash_decoder_on_level(flash_decoder_t *d, bool lit, uint32_t now_ms)
{
    settle(d, now_ms);
    if (!d->level_known) {
        d->level_known = true;
        d->level_lit = lit;
        d->last_any_edge_ms = now_ms;
    } else if (!lit) {
        if (d->level_lit && !d->pending_low) {
            d->pending_low = true;
            d->fall_raw_ms = now_ms;
        }
    } else if (d->pending_low) {
        d->pending_low = false;  /* a short low gap: chopping or a glitch */
    } else if (!d->level_lit) {
        commit_edge(d, true, now_ms);
    }
}

void flash_decoder_tick(flash_decoder_t *d, uint32_t now_ms)
{
    settle(d, now_ms);
}

gate_state_t flash_decoder_state(const flash_decoder_t *d)
{
    return d->state;
}

bool flash_decoder_motion_fresh(const flash_decoder_t *d, uint32_t now_ms)
{
    /* A motion state exists only inside a run of edges, so last_any_edge_ms is the last decoded edge. */
    uint32_t window_ms = 0;
    switch (d->state) {
    case GATE_STATE_OPENING: window_ms = GATE_FRESH_EDGE_MS(FLASH_SLOW_PERIOD_MS); break;
    case GATE_STATE_CLOSING: window_ms = GATE_FRESH_EDGE_MS(FLASH_FAST_PERIOD_MS); break;
    case GATE_STATE_UNKNOWN:
    case GATE_STATE_CLOSED:
    case GATE_STATE_NOT_CLOSED: return false;
    }
    return (uint32_t)(now_ms - d->last_any_edge_ms) <= window_ms;
}

gate_motion_t flash_decoder_last_motion(const flash_decoder_t *d)
{
    return d->last_motion;
}
