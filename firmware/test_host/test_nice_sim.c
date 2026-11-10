/* Tests of the Nice SNA2 model, driven in 1 ms steps so that every outcome is deterministic. */
#include <stdbool.h>
#include <stdint.h>

#include "flash_timing.h"
#include "minitest.h"
#include "nice_sim.h"

typedef struct {
    nice_sim_t sim;
    uint32_t now;
} fixture_t;

static void fixture_init_at(fixture_t *f, uint16_t chop_hz, uint8_t chop_duty_pct, uint32_t start_ms)
{
    const nice_sim_config_t cfg = {.travel_ms = 15000u, .chop_hz = chop_hz, .chop_duty_pct = chop_duty_pct};
    f->now = start_ms;
    nice_sim_init(&f->sim, &cfg, f->now);
}

static void fixture_init(fixture_t *f, uint16_t chop_hz, uint8_t chop_duty_pct)
{
    fixture_init_at(f, chop_hz, chop_duty_pct, 1000u);
}

static void run_ms(fixture_t *f, uint32_t duration_ms)
{
    for (uint32_t i = 0; i < duration_ms; i++) {
        f->now++;
        nice_sim_tick(&f->sim, f->now);
    }
}

/* Closes the contact for hold_ms, then releases it. */
static void press(fixture_t *f, uint32_t hold_ms)
{
    nice_sim_set_pp(&f->sim, true, f->now);
    run_ms(f, hold_ms);
    nice_sim_set_pp(&f->sim, false, f->now);
}

static bool flash_at(const fixture_t *f) { return nice_sim_flash(&f->sim, f->now); }

static void test_step_sequence_open_stop_close_stop(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSED);
    CHECK(nice_sim_position_permille(&f.sim) == 0);

    press(&f, 200);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_OPENING);
    run_ms(&f, 3000);
    press(&f, 200);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_STOPPED);
    const int32_t stop_position = nice_sim_position_permille(&f.sim);
    CHECK_NEAR(stop_position, 3200 * 1000.0 / 15000.0, 2);
    run_ms(&f, 2000);
    CHECK(nice_sim_position_permille(&f.sim) == stop_position);

    press(&f, 200);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSING);
    run_ms(&f, 2000);
    press(&f, 200);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_STOPPED);

    /* Stopped after closing: the next step opens. */
    press(&f, 200);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_OPENING);
}

static void test_travel_reaches_the_limits(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    press(&f, 200);
    run_ms(&f, 15000);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_STOPPED);
    CHECK(nice_sim_position_permille(&f.sim) == 1000);
    CHECK(flash_at(&f));

    /* Fully open: the next step closes. */
    press(&f, 200);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSING);
    run_ms(&f, 15000);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSED);
    CHECK(nice_sim_position_permille(&f.sim) == 0);
}

static void test_flash_patterns(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    CHECK(!flash_at(&f));  /* closed: low */
    run_ms(&f, 2500);
    CHECK(!flash_at(&f));

    press(&f, 200);  /* opening: slow blink starting with the high phase */
    const uint32_t open_start = f.now - 200;
    const uint32_t half = FLASH_SLOW_PERIOD_MS * FLASH_BLINK_DUTY_PCT / 100u;
    CHECK(nice_sim_flash(&f.sim, open_start));
    CHECK(nice_sim_flash(&f.sim, open_start + half - 1u));
    CHECK(!nice_sim_flash(&f.sim, open_start + half));
    CHECK(!nice_sim_flash(&f.sim, open_start + FLASH_SLOW_PERIOD_MS - 1u));
    CHECK(nice_sim_flash(&f.sim, open_start + FLASH_SLOW_PERIOD_MS));

    run_ms(&f, 1000);
    press(&f, 200);  /* stopped: steady high */
    for (int i = 0; i < 20; i++) {
        run_ms(&f, 100);
        CHECK(flash_at(&f));
    }

    press(&f, 200);  /* closing: fast blink */
    const uint32_t close_start = f.now - 200;
    const uint32_t fast_half = FLASH_FAST_PERIOD_MS * FLASH_BLINK_DUTY_PCT / 100u;
    CHECK(nice_sim_flash(&f.sim, close_start));
    CHECK(nice_sim_flash(&f.sim, close_start + fast_half - 1u));
    CHECK(!nice_sim_flash(&f.sim, close_start + fast_half));
    CHECK(!nice_sim_flash(&f.sim, close_start + FLASH_FAST_PERIOD_MS - 1u));
    CHECK(nice_sim_flash(&f.sim, close_start + FLASH_FAST_PERIOD_MS));
}

static void test_hold_over_3s_is_always_open(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    press(&f, 200);
    run_ms(&f, 15000);
    press(&f, 200);  /* closing */
    run_ms(&f, 2000);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSING);
    CHECK(!nice_sim_always_open_seen(&f.sim));

    /* The closing edge stops the gate, the 3 s mark opens it. */
    nice_sim_set_pp(&f.sim, true, f.now);
    run_ms(&f, 2900);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_STOPPED);
    CHECK(!nice_sim_always_open_seen(&f.sim));
    run_ms(&f, 200);
    nice_sim_set_pp(&f.sim, false, f.now);
    CHECK(nice_sim_always_open_seen(&f.sim));
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_OPENING);
    CHECK(nice_sim_last_pp_closure_ms(&f.sim) == 3100u);
}

static void test_hold_over_3s_opens_from_closed_and_stopped(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    press(&f, 3100);
    CHECK(nice_sim_always_open_seen(&f.sim));
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_OPENING);

    run_ms(&f, 15000);
    CHECK(nice_sim_position_permille(&f.sim) == 1000);
    press(&f, 3100);  /* fully open: the edge closes, the 3 s mark opens again */
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_OPENING);
}

static void test_hold_1500ms_is_normal_step(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    press(&f, 1500);
    CHECK(!nice_sim_always_open_seen(&f.sim));
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_OPENING);
    CHECK(nice_sim_last_pp_closure_ms(&f.sim) == 1500u);
    run_ms(&f, 2000);
    press(&f, 1500);
    CHECK(!nice_sim_always_open_seen(&f.sim));
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_STOPPED);
}

static void test_obstacle_reverses_1s_then_stops(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    press(&f, 200);
    run_ms(&f, 15000);
    press(&f, 200);
    run_ms(&f, 3000);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSING);
    const int32_t before = nice_sim_position_permille(&f.sim);

    nice_sim_obstacle(&f.sim, f.now);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_REVERSING);
    CHECK(flash_at(&f));  /* slow blink, high phase first */
    run_ms(&f, 500);
    CHECK(!flash_at(&f));
    run_ms(&f, 499);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_REVERSING);
    run_ms(&f, 1);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_STOPPED);
    CHECK_NEAR(nice_sim_position_permille(&f.sim) - before, 1000 * 1000.0 / 15000.0, 2);

    /* FLASH stays high after the reversal. */
    for (int i = 0; i < 30; i++) {
        run_ms(&f, 100);
        CHECK(flash_at(&f));
    }
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_STOPPED);
}

static void test_obstacle_is_ignored_outside_closing(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    nice_sim_obstacle(&f.sim, f.now);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSED);

    press(&f, 200);
    run_ms(&f, 3000);
    nice_sim_obstacle(&f.sim, f.now);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_OPENING);
    press(&f, 200);
    nice_sim_obstacle(&f.sim, f.now);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_STOPPED);
}

/* Counts high samples and rising edges of FLASH over the window. */
static void sample_flash(fixture_t *f, uint32_t duration_ms, uint32_t *high, uint32_t *rises)
{
    *high = 0;
    *rises = 0;
    bool previous = flash_at(f);
    for (uint32_t i = 0; i < duration_ms; i++) {
        run_ms(f, 1);
        const bool level = flash_at(f);
        *high += level ? 1u : 0u;
        *rises += (level && !previous) ? 1u : 0u;
        previous = level;
    }
}

static void test_chopped_flash(void)
{
    fixture_t f;
    fixture_init(&f, 100, 30);
    uint32_t high;
    uint32_t rises;
    sample_flash(&f, 1000, &high, &rises);  /* closed: still low */
    CHECK(high == 0u);

    press(&f, 200);
    run_ms(&f, 15000);
    press(&f, 200);
    run_ms(&f, 50);  /* closing: fast blink; the first half of every period is chopped */
    const uint32_t close_start = f.now - 250u;
    uint32_t high_in_phase = 0;
    uint32_t low_in_phase = 0;
    for (uint32_t t = 0; t < FLASH_FAST_PERIOD_MS * 5u; t++) {
        const bool level = nice_sim_flash(&f.sim, close_start + t);
        const bool lit_phase = (t % FLASH_FAST_PERIOD_MS) < FLASH_FAST_PERIOD_MS * FLASH_BLINK_DUTY_PCT / 100u;
        if (lit_phase) {
            high_in_phase += level ? 1u : 0u;
        } else {
            low_in_phase += level ? 1u : 0u;
        }
    }
    CHECK(low_in_phase == 0u);
    CHECK(high_in_phase == 5u * 200u * 30u / 100u);  /* 30 % of the 200 ms lit phases */

    press(&f, 200);  /* stopped: steady "high" is chopped at 100 Hz, 30 % */
    run_ms(&f, 1000);
    sample_flash(&f, 1000, &high, &rises);
    CHECK(high == 300u);
    CHECK(rises == 100u);
}

static void test_closure_length_measured(void)
{
    fixture_t f;
    fixture_init(&f, 0, 0);
    CHECK(nice_sim_last_pp_closure_ms(&f.sim) == 0u);
    press(&f, 500);
    CHECK(nice_sim_last_pp_closure_ms(&f.sim) == 500u);
    run_ms(&f, 100);
    press(&f, 80);
    CHECK(nice_sim_last_pp_closure_ms(&f.sim) == 80u);
    nice_sim_set_pp(&f.sim, true, f.now);
    run_ms(&f, 250);
    CHECK(nice_sim_last_pp_closure_ms(&f.sim) == 80u);  /* still closed: the old value stays */
}

typedef enum {
    START_CLOSED,
    START_OPENING,
    START_FULLY_OPEN,
    START_STOPPED_AFTER_OPENING,
    START_CLOSING,
    START_STOPPED_AFTER_CLOSING,
    START_REVERSING,
    START_COUNT
} start_t;

static void set_up_start(fixture_t *f, start_t start)
{
    if (start == START_CLOSED) {
        return;
    }
    press(f, 200);  /* opening */
    if (start == START_OPENING) {
        run_ms(f, 5000);
    } else if (start == START_STOPPED_AFTER_OPENING) {
        run_ms(f, 5000);
        press(f, 200);
    } else if (start >= START_FULLY_OPEN) {
        run_ms(f, 15000);
        if (start >= START_CLOSING) {
            press(f, 200);
            run_ms(f, 5000);
        }
        if (start == START_STOPPED_AFTER_CLOSING) {
            press(f, 200);
        } else if (start == START_REVERSING) {
            nice_sim_obstacle(&f->sim, f->now);
        }
    }
}

static void test_always_open_edge_at_3000ms_from_every_start(void)
{
    for (int s = 0; s < START_COUNT; s++) {
        fixture_t base;
        fixture_init(&base, 0, 0);
        set_up_start(&base, (start_t)s);
        CHECK(!nice_sim_always_open_seen(&base.sim));

        fixture_t short_hold = base;  /* 2999 ms is a normal step */
        nice_sim_set_pp(&short_hold.sim, true, short_hold.now);
        run_ms(&short_hold, 2999);
        nice_sim_set_pp(&short_hold.sim, false, short_hold.now);
        CHECK(!nice_sim_always_open_seen(&short_hold.sim));
        CHECK(nice_sim_last_pp_closure_ms(&short_hold.sim) == 2999u);

        fixture_t long_hold = base;  /* 3000 ms is "always open", seen by the ticks */
        nice_sim_set_pp(&long_hold.sim, true, long_hold.now);
        run_ms(&long_hold, 3000);
        CHECK(nice_sim_always_open_seen(&long_hold.sim));
        CHECK(nice_sim_state(&long_hold.sim) == NICE_SIM_OPENING);

        /* A ticker slower than the hold must not miss it: the release is checked too. */
        fixture_t coarse = base;
        nice_sim_set_pp(&coarse.sim, true, coarse.now);
        coarse.now += 3000u;
        nice_sim_set_pp(&coarse.sim, false, coarse.now);
        CHECK(nice_sim_always_open_seen(&coarse.sim));
        CHECK(nice_sim_state(&coarse.sim) == NICE_SIM_OPENING);
    }
}

static void test_time_wraps_around(void)
{
    fixture_t f;
    fixture_init_at(&f, 0, 0, UINT32_MAX - 20000u);
    press(&f, 200);
    run_ms(&f, 15000);
    CHECK(nice_sim_position_permille(&f.sim) == 1000);
    press(&f, 200);
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSING);
    run_ms(&f, 15000);  /* the 32-bit clock wraps in this window */
    CHECK(nice_sim_state(&f.sim) == NICE_SIM_CLOSED);
    CHECK(nice_sim_position_permille(&f.sim) == 0);
    press(&f, 3100);
    CHECK(nice_sim_always_open_seen(&f.sim));
}

static void test_chop_phase_survives_clock_wrap(void)
{
    const uint32_t start_ms = UINT32_MAX - 15200u - 500u;
    fixture_t f;
    fixture_init_at(&f, 100, 30, start_ms);
    press(&f, 200);
    run_ms(&f, 15000);  /* fully open: steady "high", chopped */
    CHECK(f.now > UINT32_MAX - 600u);
    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < 1000u; i++) {
        run_ms(&f, 1);
        /* The chop phase counts from the model's start, so the 32-bit wrap does not disturb it. */
        const bool expected = ((f.now - start_ms) % 10u) * 100u < 10u * 30u;
        mismatches += (flash_at(&f) != expected) ? 1u : 0u;
    }
    CHECK(f.now < 1000u);  /* the window crossed the wrap */
    CHECK(mismatches == 0u);
}

int main(void)
{
    RUN_TEST(test_step_sequence_open_stop_close_stop);
    RUN_TEST(test_travel_reaches_the_limits);
    RUN_TEST(test_flash_patterns);
    RUN_TEST(test_hold_over_3s_is_always_open);
    RUN_TEST(test_hold_over_3s_opens_from_closed_and_stopped);
    RUN_TEST(test_hold_1500ms_is_normal_step);
    RUN_TEST(test_obstacle_reverses_1s_then_stops);
    RUN_TEST(test_obstacle_is_ignored_outside_closing);
    RUN_TEST(test_chopped_flash);
    RUN_TEST(test_closure_length_measured);
    RUN_TEST(test_always_open_edge_at_3000ms_from_every_start);
    RUN_TEST(test_time_wraps_around);
    RUN_TEST(test_chop_phase_survives_clock_wrap);
    return MINITEST_RESULT();
}
