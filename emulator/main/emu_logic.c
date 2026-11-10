#include "emu_logic.h"

#include <errno.h>
#include <stdlib.h>

#define PERMILLE_FULL 1000

static bool parse_uint(const char *text, uint32_t min, uint32_t max, uint32_t *value)
{
    if (text == NULL || text[0] < '0' || text[0] > '9') {
        return false;  /* rejects "", signs and leading spaces that strtoul would accept */
    }
    char *end = NULL;
    errno = 0;
    const unsigned long parsed = strtoul(text, &end, 10);
    if (errno != 0 || *end != '\0' || parsed < min || parsed > max) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

bool emu_parse_travel(const char *text, uint32_t *travel_ms)
{
    return parse_uint(text, EMU_TRAVEL_MIN_MS, EMU_TRAVEL_MAX_MS, travel_ms);
}

bool emu_parse_chop(const char *hz, const char *duty, uint16_t *chop_hz, uint8_t *duty_pct)
{
    uint32_t parsed_hz = 0;
    if (!parse_uint(hz, 0, EMU_CHOP_MAX_HZ, &parsed_hz)) {
        return false;
    }
    if (parsed_hz == 0) {
        *chop_hz = 0;
        return true;
    }
    uint32_t parsed_duty = 0;
    if (!parse_uint(duty, 1, 99, &parsed_duty)) {
        return false;
    }
    *chop_hz = (uint16_t)parsed_hz;
    *duty_pct = (uint8_t)parsed_duty;
    return true;
}

void emu_chop_effective(uint16_t chop_hz, uint8_t duty_pct, uint32_t *period_ms, uint32_t *on_ms)
{
    const uint32_t period = 1000u / chop_hz > 0u ? 1000u / chop_hz : 1u;
    *period_ms = period;
    *on_ms = (period * duty_pct + 99u) / 100u;
}

emu_reconfig_t emu_reconfigure_check(bool model_closed, bool raw_contact_closed, bool debounced_contact_closed)
{
    if (!model_closed) {
        return EMU_RECONFIG_NOT_CLOSED;
    }
    return raw_contact_closed || debounced_contact_closed ? EMU_RECONFIG_CONTACT_HELD : EMU_RECONFIG_OK;
}

void emu_history_init(emu_history_t *h)
{
    *h = (emu_history_t){0};
}

void emu_history_record_closure(emu_history_t *h, uint32_t length_ms)
{
    h->closures++;
    h->last_closure_ms = length_ms;
}

void emu_history_note_always_open(emu_history_t *h, bool model_flag)
{
    h->always_open_seen = h->always_open_seen || model_flag;
}

int32_t emu_steps_for_permille(int32_t permille)
{
    const int32_t clamped = permille < 0 ? 0 : (permille > PERMILLE_FULL ? PERMILLE_FULL : permille);
    return clamped * EMU_STEPS_PER_TRAVEL / PERMILLE_FULL;
}

void emu_debounce_init(emu_debounce_t *d, bool initial)
{
    d->stable = initial;
    d->disagree_count = 0;
}

bool emu_debounce_update(emu_debounce_t *d, bool raw)
{
    if (raw == d->stable) {
        d->disagree_count = 0;
        return false;
    }
    if (++d->disagree_count < EMU_DEBOUNCE_SAMPLES) {
        return false;
    }
    d->stable = raw;
    d->disagree_count = 0;
    return true;
}

bool emu_debounce_stable(const emu_debounce_t *d)
{
    return d->stable;
}

void emu_glitch_init(emu_glitch_t *g)
{
    *g = (emu_glitch_t){0};
}

uint32_t emu_glitch_update(emu_glitch_t *g, bool raw_closed, bool debounced_closed)
{
    if (debounced_closed) {
        g->pending_ms = 0;
        g->open_run = 0;
        return 0;
    }
    if (raw_closed) {
        g->pending_ms++;
        g->open_run = 0;
        return 0;
    }
    if (g->pending_ms == 0 || ++g->open_run < EMU_DEBOUNCE_SAMPLES) {
        return 0;
    }
    const uint32_t closed_ms = g->pending_ms;
    g->pending_ms = 0;
    g->open_run = 0;
    g->count++;
    return closed_ms;
}

uint32_t emu_glitch_count(const emu_glitch_t *g)
{
    return g->count;
}

void emu_stepper_init(emu_stepper_t *st)
{
    st->position = 0;
    st->step_high = false;
    st->dir_open = false;
    st->enabled = false;
    st->idle_ticks = 0;
}

static emu_stepper_out_t outputs(const emu_stepper_t *st)
{
    const emu_stepper_out_t out = {.step = st->step_high, .dir_open = st->dir_open, .enable = st->enabled};
    return out;
}

emu_stepper_out_t emu_stepper_tick(emu_stepper_t *st, int32_t target)
{
    if (st->step_high) {
        st->step_high = false;
    } else if (st->position == target) {
        if (st->enabled && ++st->idle_ticks >= EMU_STEPPER_HOLD_TICKS) {
            st->enabled = false;
        }
    } else {
        st->idle_ticks = 0;
        const bool want_open = target > st->position;
        if (!st->enabled) {
            st->enabled = true;
        } else if (st->dir_open != want_open) {
            st->dir_open = want_open;
        } else {
            st->step_high = true;
            st->position += want_open ? 1 : -1;
        }
    }
    return outputs(st);
}
