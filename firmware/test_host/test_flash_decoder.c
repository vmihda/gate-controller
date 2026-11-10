// Host tests for the FLASH signal decoder. Time is simulated in 1 ms steps.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flash_decoder.h"
#include "gate_config.h"
#include "minitest.h"

#define STATE_COUNT 5u
#define TICK_PERIOD_MS 10u
#define RECOGNITION_LIMIT_MS 2000u
#define SETTLE_MARGIN_MS 10u
#define LONG_RUN_MS 6000u
#define WRAP_MARGIN_MS 3000u
#define STOP_STEP_MS 5u   // stop moments swept over one blink period
#define PHASE_STEPS 20u   // blink phases swept per period

static uint32_t g_t0;

typedef struct {
    bool constant;
    bool lit;            // level of a constant signal
    uint32_t origin_ms;  // absolute time where the signal starts
    uint32_t period_ms;  // periodic signal: lit while (elapsed + shift) % period < on_ms
    uint32_t on_ms;
    uint32_t shift_ms;
    uint32_t glitch_at_ms;  // constant lit signal: low for glitch_len_ms at this elapsed time
    uint32_t glitch_len_ms;
    uint32_t chop_period_ms;  // 0 = no chopping; else lit phases are lit only while
    uint32_t chop_on_ms;      // (elapsed + chop_shift_ms) % chop_period_ms < chop_on_ms
    uint32_t chop_shift_ms;
    bool stops;            // periodic signal: from stop_at_ms on it is steady at stop_lit
    uint32_t stop_at_ms;
    bool stop_lit;
} signal_t;

typedef struct {
    flash_decoder_t decoder;
    uint32_t start_ms;
    uint32_t now_ms;
    uint32_t mark_ms;
    bool raw_known;
    bool raw_lit;
    bool seen[STATE_COUNT];
    uint32_t first_ms[STATE_COUNT];  // elapsed since mark when a state was first seen
    unsigned stale_motion_ticks;     // ticks since mark with a motion state that is not fresh
} rig_t;

static void rig_init(rig_t *r, uint32_t t0)
{
    memset(r, 0, sizeof *r);
    r->start_ms = t0;
    r->now_ms = t0;
    r->mark_ms = t0;
    flash_decoder_init(&r->decoder, t0);
}

static void rig_mark(rig_t *r)
{
    r->mark_ms = r->now_ms;
    memset(r->seen, 0, sizeof r->seen);
    memset(r->first_ms, 0, sizeof r->first_ms);
    r->stale_motion_ticks = 0;
}

static signal_t sig_steady(const rig_t *r, bool lit)
{
    signal_t s;
    memset(&s, 0, sizeof s);
    s.constant = true;
    s.lit = lit;
    s.origin_ms = r->now_ms;
    return s;
}

static signal_t sig_periodic(const rig_t *r, uint32_t period_ms, uint32_t on_ms, uint32_t shift_ms)
{
    signal_t s = sig_steady(r, false);
    s.constant = false;
    s.period_ms = period_ms;
    s.on_ms = on_ms;
    s.shift_ms = shift_ms;
    return s;
}

static signal_t sig_blink(const rig_t *r, uint32_t period_ms, uint32_t shift_ms)
{
    return sig_periodic(r, period_ms, period_ms / 2u, shift_ms);
}

static signal_t sig_chopped(signal_t s, uint32_t chop_period_ms, uint32_t chop_on_ms, uint32_t chop_shift_ms)
{
    s.chop_period_ms = chop_period_ms;
    s.chop_on_ms = chop_on_ms;
    s.chop_shift_ms = chop_shift_ms;
    return s;
}

static signal_t sig_stopping(signal_t s, uint32_t stop_at_ms, bool stop_lit)
{
    s.stops = true;
    s.stop_at_ms = stop_at_ms;
    s.stop_lit = stop_lit;
    return s;
}

// Level of the signal before chopping.
static bool sig_unchopped(const signal_t *s, uint32_t elapsed)
{
    if (s->constant) {
        return s->lit && !(s->glitch_len_ms > 0u && (uint32_t)(elapsed - s->glitch_at_ms) < s->glitch_len_ms);
    }
    if (s->stops && elapsed >= s->stop_at_ms) {
        return s->stop_lit;
    }
    return ((elapsed + s->shift_ms) % s->period_ms) < s->on_ms;
}

static bool sig_level(const signal_t *s, uint32_t now_ms)
{
    const uint32_t elapsed = (uint32_t)(now_ms - s->origin_ms);
    const bool chop_lit = s->chop_period_ms == 0u || ((elapsed + s->chop_shift_ms) % s->chop_period_ms) < s->chop_on_ms;
    return sig_unchopped(s, elapsed) && chop_lit;
}

// Elapsed time of the first unchopped level different from `from_lit`: the moment FLASH changes.
static uint32_t sig_first_change_ms(const signal_t *s, bool from_lit)
{
    uint32_t elapsed = 0;
    while (sig_unchopped(s, elapsed) == from_lit) {
        elapsed++;
    }
    return elapsed;
}

static void rig_step(rig_t *r, const signal_t *s)
{
    const bool lit = sig_level(s, r->now_ms);
    if (!r->raw_known || lit != r->raw_lit) {
        flash_decoder_on_level(&r->decoder, lit, r->now_ms);
        r->raw_known = true;
        r->raw_lit = lit;
    }

    if ((uint32_t)(r->now_ms - r->start_ms) % TICK_PERIOD_MS == 0u) {
        flash_decoder_tick(&r->decoder, r->now_ms);
        const gate_state_t ticked = flash_decoder_state(&r->decoder);
        if ((ticked == GATE_STATE_OPENING || ticked == GATE_STATE_CLOSING) &&
            !flash_decoder_motion_fresh(&r->decoder, r->now_ms)) {
            r->stale_motion_ticks++;
        }
    }
    const gate_state_t state = flash_decoder_state(&r->decoder);
    if (!r->seen[state]) {
        r->seen[state] = true;
        r->first_ms[state] = (uint32_t)(r->now_ms - r->mark_ms);
    }
    r->now_ms++;
}

static void rig_run(rig_t *r, const signal_t *s, uint32_t duration_ms)
{
    for (uint32_t i = 0; i < duration_ms; i++) {
        rig_step(r, s);
    }
}

static gate_state_t rig_state(const rig_t *r)
{
    return flash_decoder_state(&r->decoder);
}

static void test_starts_unknown(void)
{
    flash_decoder_t d;
    flash_decoder_init(&d, g_t0);
    CHECK(flash_decoder_state(&d) == GATE_STATE_UNKNOWN);
    CHECK(flash_decoder_last_motion(&d) == GATE_MOTION_NONE);
}

static void check_steady_level(bool lit, gate_state_t expected)
{
    rig_t r;
    rig_init(&r, g_t0);
    const signal_t s = sig_steady(&r, lit);
    rig_run(&r, &s, GATE_STEADY_AFTER_MS - SETTLE_MARGIN_MS);
    CHECK(rig_state(&r) == GATE_STATE_UNKNOWN);
    rig_run(&r, &s, 2u * SETTLE_MARGIN_MS + 1u);
    CHECK(rig_state(&r) == expected);
}

static void test_steady_low_becomes_closed(void)
{
    check_steady_level(false, GATE_STATE_CLOSED);
}

static void test_steady_high_becomes_not_closed(void)
{
    check_steady_level(true, GATE_STATE_NOT_CLOSED);
}

static void check_blink_recognized(uint32_t period_ms, gate_state_t expected)
{
    rig_t r;
    rig_init(&r, g_t0);
    rig_mark(&r);
    const signal_t s = sig_blink(&r, period_ms, 0u);
    rig_run(&r, &s, LONG_RUN_MS / 2u);
    CHECK(r.seen[expected]);
    CHECK(r.first_ms[expected] <= RECOGNITION_LIMIT_MS);
    CHECK(rig_state(&r) == expected);
}

static void test_slow_blink_is_opening(void)
{
    check_blink_recognized(FLASH_SLOW_PERIOD_MS, GATE_STATE_OPENING);
}

static void test_fast_blink_is_closing(void)
{
    check_blink_recognized(FLASH_FAST_PERIOD_MS, GATE_STATE_CLOSING);
}

static void check_period_result(uint32_t period_ms, gate_state_t expected)
{
    rig_t r;
    rig_init(&r, g_t0);
    const signal_t s = sig_blink(&r, period_ms, 0u);
    rig_run(&r, &s, LONG_RUN_MS);
    CHECK(rig_state(&r) == expected);
    if (expected == GATE_STATE_UNKNOWN) {
        CHECK(!r.seen[GATE_STATE_NOT_CLOSED]);
    }
}

static void test_period_at_tolerance_edges(void)
{
    // Inclusive bounds: x0.75 and x1.25 of each nominal period are recognized.
    check_period_result(FLASH_SLOW_PERIOD_MS * 75u / 100u, GATE_STATE_OPENING);
    check_period_result(FLASH_SLOW_PERIOD_MS * 125u / 100u, GATE_STATE_OPENING);
    check_period_result(FLASH_FAST_PERIOD_MS * 75u / 100u, GATE_STATE_CLOSING);
    check_period_result(FLASH_FAST_PERIOD_MS * 125u / 100u, GATE_STATE_CLOSING);
    // Between the bands, below the fast band and above the slow band.
    check_period_result(FLASH_SLOW_PERIOD_MS * 6u / 10u, GATE_STATE_UNKNOWN);
    check_period_result(FLASH_FAST_PERIOD_MS * 5u / 10u, GATE_STATE_UNKNOWN);
    check_period_result(FLASH_SLOW_PERIOD_MS * 16u / 10u, GATE_STATE_UNKNOWN);
}

static void test_single_off_phase_is_not_closed(void)
{
    rig_t r;
    rig_init(&r, g_t0);
    const signal_t s = sig_blink(&r, FLASH_SLOW_PERIOD_MS, 0u);
    rig_run(&r, &s, LONG_RUN_MS);
    CHECK(!r.seen[GATE_STATE_CLOSED]);
    CHECK(!r.seen[GATE_STATE_NOT_CLOSED]);
}

static void check_chopped_high(uint32_t period_ms, uint32_t on_ms)
{
    rig_t r;
    rig_init(&r, g_t0);
    const signal_t s = sig_periodic(&r, period_ms, on_ms, 0u);
    rig_run(&r, &s, 3u * GATE_STEADY_AFTER_MS);
    CHECK(rig_state(&r) == GATE_STATE_NOT_CLOSED);
    CHECK(!r.seen[GATE_STATE_CLOSED]);
}

static void test_chopped_high_reads_steady(void)
{
    check_chopped_high(10u, 3u);   // 100 Hz, 30 % duty: 7 ms gaps
    check_chopped_high(20u, 10u);  // 50 Hz, 50 % duty: 10 ms gaps
}

static void test_single_glitch_ignored(void)
{
    rig_t r;
    rig_init(&r, g_t0);
    signal_t s = sig_steady(&r, true);
    rig_run(&r, &s, 2u * GATE_STEADY_AFTER_MS);
    CHECK(rig_state(&r) == GATE_STATE_NOT_CLOSED);

    rig_mark(&r);
    s = sig_steady(&r, true);
    s.glitch_at_ms = 100u;
    s.glitch_len_ms = 5u;
    rig_run(&r, &s, 500u);
    CHECK(rig_state(&r) == GATE_STATE_NOT_CLOSED);
    CHECK(!r.seen[GATE_STATE_UNKNOWN]);
    CHECK(!r.seen[GATE_STATE_CLOSED]);
}

static void check_stop_motion(uint32_t period_ms, uint32_t blink_ms, bool stop_lit, gate_state_t expected_state,
                              gate_motion_t expected_motion)
{
    rig_t r;
    rig_init(&r, g_t0);
    signal_t s = sig_blink(&r, period_ms, 0u);
    rig_run(&r, &s, blink_ms);
    s = sig_steady(&r, stop_lit);
    rig_run(&r, &s, 2u * GATE_STEADY_AFTER_MS);
    CHECK(rig_state(&r) == expected_state);
    CHECK(flash_decoder_last_motion(&r.decoder) == expected_motion);
}

static void test_last_motion_after_stop(void)
{
    const uint32_t fast = FLASH_FAST_PERIOD_MS;
    const uint32_t slow = FLASH_SLOW_PERIOD_MS;
    const uint32_t partial_ms = 50u;
    // Phase-aligned stop.
    check_stop_motion(fast, 5u * fast, true, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING);
    // Stop partial_ms into an off phase (truncated last period), steady on.
    check_stop_motion(fast, 4u * fast + fast / 2u + partial_ms, true, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING);
    check_stop_motion(slow, 3u * slow + slow / 2u + partial_ms, true, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING);
    // Stop partial_ms into an on phase (truncated on phase), steady off.
    check_stop_motion(fast, 4u * fast + partial_ms, false, GATE_STATE_CLOSED, GATE_MOTION_CLOSING);
    check_stop_motion(slow, 3u * slow + partial_ms, false, GATE_STATE_CLOSED, GATE_MOTION_OPENING);
}

static void test_decoder_needs_level_seed(void)
{
    flash_decoder_t d;
    flash_decoder_init(&d, g_t0);
    for (uint32_t t = 0; t <= 5000u; t += TICK_PERIOD_MS) {
        flash_decoder_tick(&d, g_t0 + t);
        CHECK(flash_decoder_state(&d) == GATE_STATE_UNKNOWN);
    }

    const bool levels[] = {false, true};
    const gate_state_t expected[] = {GATE_STATE_CLOSED, GATE_STATE_NOT_CLOSED};
    for (size_t i = 0; i < 2u; i++) {
        flash_decoder_init(&d, g_t0);
        flash_decoder_on_level(&d, levels[i], g_t0);
        for (uint32_t t = 0; t <= GATE_STEADY_AFTER_MS + SETTLE_MARGIN_MS; t += TICK_PERIOD_MS) {
            flash_decoder_tick(&d, g_t0 + t);
        }
        CHECK(flash_decoder_state(&d) == expected[i]);
    }
}

static void test_change_reflected_within_2s(void)
{
    const uint32_t shifts[] = {0u, 250u, 500u, 750u};
    for (size_t i = 0; i < sizeof shifts / sizeof shifts[0]; i++) {
        rig_t r;
        rig_init(&r, g_t0);
        signal_t s = sig_steady(&r, false);
        rig_run(&r, &s, GATE_STEADY_AFTER_MS + 100u);
        CHECK(rig_state(&r) == GATE_STATE_CLOSED);

        rig_mark(&r);
        s = sig_blink(&r, FLASH_SLOW_PERIOD_MS, shifts[i]);
        rig_run(&r, &s, LONG_RUN_MS / 2u);
        CHECK(r.seen[GATE_STATE_OPENING]);
        CHECK(r.first_ms[GATE_STATE_OPENING] <= RECOGNITION_LIMIT_MS);
    }
}

// Review C1: a 750 ms opening blink rises at 9500 and falls at 9875; a wall stop lights FLASH at 9991.
// The last rise-to-rise interval, 491 ms, lies in the fast band, but its lit half is a slow one.
static void test_truncated_slow_period_at_wall_stop_is_not_closing(void)
{
    const uint32_t blink_start_ms = 2000u;
    const uint32_t stop_ms = 9991u;
    const uint32_t period_ms = FLASH_SLOW_PERIOD_MS * 75u / 100u;
    rig_t r;
    rig_init(&r, g_t0);
    signal_t s = sig_steady(&r, false);
    rig_run(&r, &s, blink_start_ms);
    rig_mark(&r);
    s = sig_stopping(sig_periodic(&r, period_ms, period_ms / 2u, 0u), stop_ms - blink_start_ms, true);
    rig_run(&r, &s, LONG_RUN_MS + 4000u);
    CHECK(r.seen[GATE_STATE_OPENING]);
    CHECK(!r.seen[GATE_STATE_CLOSING]);
    CHECK(rig_state(&r) == GATE_STATE_NOT_CLOSED);
    CHECK(flash_decoder_last_motion(&r.decoder) == GATE_MOTION_OPENING);
}

// A stop at any moment of the blink, lit (stopped or open) or dark (closed), never shows the other
// motion: a truncated last period must not be read as the other blink.
// The motion reading stays fresh on every tick while the gate blinks, and stops being fresh at most one
// fresh window (plus a tick) after the stop.
static void check_stop_never_reverses(uint32_t period_ms, gate_state_t motion, gate_state_t opposite)
{
    const uint32_t fresh_window_ms =
        GATE_FRESH_EDGE_MS(motion == GATE_STATE_OPENING ? FLASH_SLOW_PERIOD_MS : FLASH_FAST_PERIOD_MS);
    const bool stop_levels[] = {true, false};
    for (uint32_t stop = 3u * period_ms; stop < 4u * period_ms; stop += STOP_STEP_MS) {
        for (size_t i = 0; i < sizeof stop_levels / sizeof stop_levels[0]; i++) {
            rig_t r;
            rig_init(&r, g_t0);
            signal_t s = sig_steady(&r, false);
            rig_run(&r, &s, GATE_STEADY_AFTER_MS + 100u);
            rig_mark(&r);
            s = sig_stopping(sig_blink(&r, period_ms, 0u), stop, stop_levels[i]);
            rig_run(&r, &s, stop);
            const int failures_before = minitest_failures;
            CHECK(r.stale_motion_ticks == 0u);  // fresh on every tick while it blinks
            rig_run(&r, &s, fresh_window_ms + TICK_PERIOD_MS);
            CHECK(!flash_decoder_motion_fresh(&r.decoder, r.now_ms));  // stopped: no longer fresh
            rig_run(&r, &s, 2u * GATE_STEADY_AFTER_MS);
            CHECK(r.seen[motion]);
            CHECK(!r.seen[opposite]);
            CHECK(rig_state(&r) == (stop_levels[i] ? GATE_STATE_NOT_CLOSED : GATE_STATE_CLOSED));
            if (minitest_failures != failures_before) {
                printf("  period %u ms, stop at %u ms, stop lit %d\n", (unsigned)period_ms, (unsigned)stop,
                       (int)stop_levels[i]);
            }
        }
    }
}

static void test_stop_at_any_phase_never_shows_the_other_motion(void)
{
    const uint32_t percents[] = {75u, 100u, 125u};
    for (size_t i = 0; i < sizeof percents / sizeof percents[0]; i++) {
        check_stop_never_reverses(FLASH_SLOW_PERIOD_MS * percents[i] / 100u, GATE_STATE_OPENING, GATE_STATE_CLOSING);
        check_stop_never_reverses(FLASH_FAST_PERIOD_MS * percents[i] / 100u, GATE_STATE_CLOSING, GATE_STATE_OPENING);
    }
}

// Review I1: seeded lit at 0, lit(t) = ((t + 10) % 1250 < 625) && (t % 20 < 10). A slow blink at the
// upper tolerance edge with 50 Hz chopping and 10 ms gaps. A command waiting at power-up gives up at
// GATE_UNKNOWN_WAIT_MS, so the state must be there one tick before.
static void test_chopped_blink_at_slow_edge_recognized_in_time(void)
{
    const uint32_t period_ms = FLASH_SLOW_PERIOD_MS * 125u / 100u;
    rig_t r;
    rig_init(&r, g_t0);
    rig_mark(&r);
    const signal_t s = sig_chopped(sig_periodic(&r, period_ms, period_ms / 2u, 10u), 20u, 10u, 0u);
    rig_run(&r, &s, LONG_RUN_MS);
    CHECK(r.seen[GATE_STATE_OPENING]);
    CHECK(r.first_ms[GATE_STATE_OPENING] <= GATE_UNKNOWN_WAIT_MS - TICK_PERIOD_MS);
    CHECK(!r.seen[GATE_STATE_CLOSING]);
    CHECK(rig_state(&r) == GATE_STATE_OPENING);
}

// Recognition deadline over the tolerance band, the chopping envelope and every phase of the blink.
typedef enum { START_POWER_UP, START_CLOSED, START_NOT_CLOSED } start_t;

typedef struct {
    uint32_t period_ms;  // 0 = no chopping
    uint32_t on_ms;
} chop_t;

// Inside the spec envelope: none, 100 Hz at 30 % (7 ms gaps), 50 Hz at 50 % (10 ms gaps).
static const chop_t CHOPS[] = {{0u, 0u}, {10u, 3u}, {20u, 10u}};
static const uint32_t CHOP_SHIFTS_MS[] = {0u, 7u};

// Power-up: the command started with the decoder waits at most GATE_UNKNOWN_WAIT_MS.
// From a steady state: Home must show the change within RECOGNITION_LIMIT_MS of the FLASH change.
// Both are checked one tick early: the device acts on the state at its 10 ms step.
static void check_deadline(uint32_t period_ms, gate_state_t expected, gate_state_t opposite, start_t start)
{
    for (size_t c = 0; c < sizeof CHOPS / sizeof CHOPS[0]; c++) {
        for (size_t k = 0; k < sizeof CHOP_SHIFTS_MS / sizeof CHOP_SHIFTS_MS[0]; k++) {
            for (uint32_t step = 0; step < PHASE_STEPS; step++) {
                const uint32_t shift = step * period_ms / PHASE_STEPS;
                rig_t r;
                rig_init(&r, g_t0);
                const bool start_lit = start == START_NOT_CLOSED;
                if (start != START_POWER_UP) {
                    const signal_t steady = sig_chopped(sig_steady(&r, start_lit), CHOPS[c].period_ms, CHOPS[c].on_ms,
                                                        CHOP_SHIFTS_MS[k]);
                    rig_run(&r, &steady, GATE_STEADY_AFTER_MS + 100u);
                    CHECK(rig_state(&r) == (start_lit ? GATE_STATE_NOT_CLOSED : GATE_STATE_CLOSED));
                }
                rig_mark(&r);
                const signal_t s = sig_chopped(sig_blink(&r, period_ms, shift), CHOPS[c].period_ms, CHOPS[c].on_ms,
                                               CHOP_SHIFTS_MS[k]);
                const uint32_t from_ms = start == START_POWER_UP ? 0u : sig_first_change_ms(&s, start_lit);
                const uint32_t limit_ms = start == START_POWER_UP ? GATE_UNKNOWN_WAIT_MS : RECOGNITION_LIMIT_MS;
                rig_run(&r, &s, LONG_RUN_MS / 2u);

                const int failures_before = minitest_failures;
                CHECK(r.seen[expected]);
                CHECK(r.first_ms[expected] - from_ms <= limit_ms - TICK_PERIOD_MS);
                CHECK(!r.seen[opposite]);
                CHECK(r.stale_motion_ticks == 0u);  // a blinking gate always reads fresh
                if (minitest_failures != failures_before) {
                    printf("  period %u ms, shift %u ms, chop %u/%u ms +%u, start %d: first at %u ms, change at %u ms\n",
                           (unsigned)period_ms, (unsigned)shift, (unsigned)CHOPS[c].on_ms, (unsigned)CHOPS[c].period_ms,
                           (unsigned)CHOP_SHIFTS_MS[k], (int)start, (unsigned)r.first_ms[expected], (unsigned)from_ms);
                }
            }
        }
    }
}

static void test_recognition_deadline_over_tolerance_chop_and_phase(void)
{
    const uint32_t percents[] = {75u, 100u, 125u};
    const start_t starts[] = {START_POWER_UP, START_CLOSED, START_NOT_CLOSED};
    for (size_t i = 0; i < sizeof percents / sizeof percents[0]; i++) {
        for (size_t j = 0; j < sizeof starts / sizeof starts[0]; j++) {
            check_deadline(FLASH_SLOW_PERIOD_MS * percents[i] / 100u, GATE_STATE_OPENING, GATE_STATE_CLOSING, starts[j]);
            check_deadline(FLASH_FAST_PERIOD_MS * percents[i] / 100u, GATE_STATE_CLOSING, GATE_STATE_OPENING, starts[j]);
        }
    }
}

int main(void)
{
    const uint32_t starts[] = {0u, UINT32_MAX - WRAP_MARGIN_MS};
    for (size_t i = 0; i < sizeof starts / sizeof starts[0]; i++) {
        g_t0 = starts[i];
        RUN_TEST(test_starts_unknown);
        RUN_TEST(test_steady_low_becomes_closed);
        RUN_TEST(test_steady_high_becomes_not_closed);
        RUN_TEST(test_slow_blink_is_opening);
        RUN_TEST(test_fast_blink_is_closing);
        RUN_TEST(test_period_at_tolerance_edges);
        RUN_TEST(test_single_off_phase_is_not_closed);
        RUN_TEST(test_chopped_high_reads_steady);
        RUN_TEST(test_single_glitch_ignored);
        RUN_TEST(test_last_motion_after_stop);
        RUN_TEST(test_decoder_needs_level_seed);
        RUN_TEST(test_change_reflected_within_2s);
        RUN_TEST(test_truncated_slow_period_at_wall_stop_is_not_closing);
        RUN_TEST(test_stop_at_any_phase_never_shows_the_other_motion);
        RUN_TEST(test_chopped_blink_at_slow_edge_recognized_in_time);
        RUN_TEST(test_recognition_deadline_over_tolerance_chop_and_phase);
    }
    return MINITEST_RESULT();
}
