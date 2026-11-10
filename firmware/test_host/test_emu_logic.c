// Host tests for the pure logic of the bench emulator: console argument parsing, debounce and the step generator.
#include <stdbool.h>
#include <stdint.h>

#include "emu_logic.h"
#include "minitest.h"

static void test_travel_accepts_a_value_inside_the_bounds(void) {
    uint32_t ms = 0;
    CHECK(emu_parse_travel("15000", &ms));
    CHECK(ms == 15000u);
    CHECK(emu_parse_travel("5000", &ms));
    CHECK(emu_parse_travel("60000", &ms));
}

static void test_travel_rejects_out_of_range_and_garbage(void) {
    uint32_t ms = 777;
    CHECK(!emu_parse_travel("4999", &ms));
    CHECK(!emu_parse_travel("60001", &ms));
    CHECK(!emu_parse_travel("-1", &ms));
    CHECK(!emu_parse_travel("12x", &ms));
    CHECK(!emu_parse_travel("", &ms));
    CHECK(!emu_parse_travel(" 15000", &ms));
    CHECK(!emu_parse_travel("99999999999999999999", &ms));
    CHECK(!emu_parse_travel(NULL, &ms));
    CHECK(ms == 777u);  // a rejected input leaves the output alone
}

static void test_chop_accepts_hz_and_duty(void) {
    uint16_t hz = 0;
    uint8_t duty = 0;
    CHECK(emu_parse_chop("100", "30", &hz, &duty));
    CHECK(hz == 100u);
    CHECK(duty == 30u);
    CHECK(emu_parse_chop("100", "99", &hz, &duty));
    CHECK(emu_parse_chop("1", "1", &hz, &duty));
}

static void test_chop_zero_means_steady_and_ignores_duty(void) {
    uint16_t hz = 55;
    uint8_t duty = 42;
    CHECK(emu_parse_chop("0", NULL, &hz, &duty));
    CHECK(hz == 0u);
    CHECK(duty == 42u);
    CHECK(emu_parse_chop("0", "0", &hz, &duty));
    CHECK(duty == 42u);
}

static void test_chop_rejects_bad_hz_and_duty(void) {
    uint16_t hz = 55;
    uint8_t duty = 42;
    CHECK(!emu_parse_chop("101", "30", &hz, &duty));
    CHECK(!emu_parse_chop("100", "0", &hz, &duty));
    CHECK(!emu_parse_chop("100", "100", &hz, &duty));
    CHECK(!emu_parse_chop("100", NULL, &hz, &duty));
    CHECK(!emu_parse_chop("abc", "30", &hz, &duty));
    CHECK(!emu_parse_chop("100", "3o", &hz, &duty));
    CHECK(!emu_parse_chop("-5", "30", &hz, &duty));
    CHECK(!emu_parse_chop(NULL, "30", &hz, &duty));
    CHECK(hz == 55u);
    CHECK(duty == 42u);
}

static void test_chop_effective_period_and_on_time_follow_whole_milliseconds(void) {
    uint32_t period = 0;
    uint32_t on = 0;
    emu_chop_effective(100, 30, &period, &on);
    CHECK(period == 10u);
    CHECK(on == 3u);
    emu_chop_effective(50, 50, &period, &on);
    CHECK(period == 20u);
    CHECK(on == 10u);
    emu_chop_effective(70, 30, &period, &on);  // 1000 / 70 rounds down to 14 ms; 30 % of 14 ms needs 5 ms
    CHECK(period == 14u);
    CHECK(on == 5u);
}

// Feeds one raw sample through the debounce and the glitch observer, as the model task does.
typedef struct {
    emu_debounce_t debounce;
    emu_glitch_t glitch;
    uint32_t closures;
    uint32_t glitch_ms_total;
    uint32_t glitches;
} observed_input_t;

static void observed_init(observed_input_t *o) {
    emu_debounce_init(&o->debounce, false);
    emu_glitch_init(&o->glitch);
    o->closures = 0;
    o->glitch_ms_total = 0;
    o->glitches = 0;
}

static void observed_feed(observed_input_t *o, bool raw, uint32_t samples) {
    for (uint32_t i = 0; i < samples; i++) {
        const bool changed = emu_debounce_update(&o->debounce, raw);
        if (changed && emu_debounce_stable(&o->debounce)) {
            o->closures++;
        }
        const uint32_t g = emu_glitch_update(&o->glitch, raw, emu_debounce_stable(&o->debounce));
        if (g > 0u) {
            o->glitches++;
            o->glitch_ms_total += g;
        }
    }
}

static void test_glitch_reports_a_two_ms_blip(void) {
    observed_input_t o;
    observed_init(&o);
    observed_feed(&o, false, 50);
    observed_feed(&o, true, 2);
    observed_feed(&o, false, 50);
    CHECK(o.closures == 0u);
    CHECK(o.glitches == 1u);
    CHECK(o.glitch_ms_total == 2u);
    CHECK(emu_glitch_count(&o.glitch) == 1u);
}

static void test_glitch_reports_chatter_as_one_episode_with_its_closed_time(void) {
    observed_input_t o;
    observed_init(&o);
    for (int i = 0; i < 10; i++) {  // 3 ms closed, 2 ms open, never 5 ms in a row
        observed_feed(&o, true, 3);
        observed_feed(&o, false, 2);
    }
    observed_feed(&o, false, 50);
    CHECK(o.closures == 0u);
    CHECK(o.glitches == 1u);
    CHECK(o.glitch_ms_total == 30u);
}

static void test_glitch_ignores_a_clean_closure_with_bounce_on_both_edges(void) {
    observed_input_t o;
    observed_init(&o);
    observed_feed(&o, false, 50);
    // Leading bounce, then 500 ms closed with a bounce on release.
    observed_feed(&o, true, 1);
    observed_feed(&o, false, 1);
    observed_feed(&o, true, 2);
    observed_feed(&o, false, 1);
    observed_feed(&o, true, 500);
    observed_feed(&o, false, 2);
    observed_feed(&o, true, 1);
    observed_feed(&o, false, 1);
    observed_feed(&o, true, 1);
    observed_feed(&o, false, 100);
    CHECK(o.closures == 1u);
    CHECK(o.glitches == 0u);
    CHECK(emu_glitch_count(&o.glitch) == 0u);
}

static void test_glitch_after_a_clean_closure_is_still_reported(void) {
    observed_input_t o;
    observed_init(&o);
    observed_feed(&o, true, 500);
    observed_feed(&o, false, 20);
    observed_feed(&o, true, 2);
    observed_feed(&o, false, 20);
    CHECK(o.closures == 1u);
    CHECK(o.glitches == 1u);
    CHECK(o.glitch_ms_total == 2u);
}

static void test_glitch_waits_for_the_input_to_settle_before_reporting(void) {
    emu_glitch_t g;
    emu_glitch_init(&g);
    CHECK(emu_glitch_update(&g, true, false) == 0u);
    CHECK(emu_glitch_update(&g, true, false) == 0u);
    for (uint32_t i = 0; i < EMU_DEBOUNCE_SAMPLES - 1u; i++) {
        CHECK(emu_glitch_update(&g, false, false) == 0u);
    }
    CHECK(emu_glitch_update(&g, false, false) == 2u);
    CHECK(emu_glitch_update(&g, false, false) == 0u);
}

static void test_reconfigure_is_allowed_only_when_closed_and_the_contact_is_released(void) {
    CHECK(emu_reconfigure_check(true, false, false) == EMU_RECONFIG_OK);
    CHECK(emu_reconfigure_check(false, false, false) == EMU_RECONFIG_NOT_CLOSED);
    CHECK(emu_reconfigure_check(false, true, true) == EMU_RECONFIG_NOT_CLOSED);
}

static void test_reconfigure_is_refused_while_the_contact_is_held_raw_or_debounced(void) {
    // CLOSED does not mean released: the gate can reach the closed limit while PP is still held.
    CHECK(emu_reconfigure_check(true, true, false) == EMU_RECONFIG_CONTACT_HELD);
    CHECK(emu_reconfigure_check(true, false, true) == EMU_RECONFIG_CONTACT_HELD);
    CHECK(emu_reconfigure_check(true, true, true) == EMU_RECONFIG_CONTACT_HELD);
}

static void test_history_counts_closures_and_keeps_the_last_length(void) {
    emu_history_t h;
    emu_history_init(&h);
    CHECK(h.closures == 0u);
    emu_history_record_closure(&h, 500u);
    emu_history_record_closure(&h, 510u);
    CHECK(h.closures == 2u);
    CHECK(h.last_closure_ms == 510u);
}

static void test_history_always_open_flag_is_sticky(void) {
    emu_history_t h;
    emu_history_init(&h);
    emu_history_note_always_open(&h, false);
    CHECK(!h.always_open_seen);
    emu_history_note_always_open(&h, true);
    emu_history_note_always_open(&h, false);  // the model was re-initialised: the history keeps the flag
    CHECK(h.always_open_seen);
}

static void test_steps_follow_position_permille(void) {
    CHECK(emu_steps_for_permille(0) == 0);
    CHECK(emu_steps_for_permille(1000) == EMU_STEPS_PER_TRAVEL);
    CHECK(emu_steps_for_permille(500) == EMU_STEPS_PER_TRAVEL / 2);
    CHECK(emu_steps_for_permille(-5) == 0);
    CHECK(emu_steps_for_permille(2000) == EMU_STEPS_PER_TRAVEL);
}

static void test_step_rate_at_the_shortest_travel_stays_below_2khz(void) {
    const double steps_per_second = (double)EMU_STEPS_PER_TRAVEL * 1000.0 / EMU_TRAVEL_MIN_MS;
    CHECK(steps_per_second <= 2000.0);
    // The generator can emit one step per two ticks of EMU_STEPPER_TICK_US.
    CHECK(1000000.0 / (2.0 * EMU_STEPPER_TICK_US) >= steps_per_second);
}

static void test_debounce_ignores_a_glitch_shorter_than_the_window(void) {
    emu_debounce_t d;
    emu_debounce_init(&d, false);
    for (uint32_t i = 0; i < EMU_DEBOUNCE_SAMPLES - 1u; i++) {
        CHECK(!emu_debounce_update(&d, true));
    }
    CHECK(!emu_debounce_update(&d, false));
    CHECK(!emu_debounce_stable(&d));
    // The glitch restarted the count: a full window is needed again.
    for (uint32_t i = 0; i < EMU_DEBOUNCE_SAMPLES - 1u; i++) {
        CHECK(!emu_debounce_update(&d, true));
    }
    CHECK(emu_debounce_update(&d, true));
    CHECK(emu_debounce_stable(&d));
}

static void test_debounce_reports_each_change_once(void) {
    emu_debounce_t d;
    emu_debounce_init(&d, true);
    int changes = 0;
    for (uint32_t i = 0; i < 20u; i++) {
        changes += emu_debounce_update(&d, false) ? 1 : 0;
    }
    CHECK(changes == 1);
    CHECK(!emu_debounce_stable(&d));
}

static void test_debounce_delays_both_edges_by_the_same_number_of_samples(void) {
    emu_debounce_t d;
    emu_debounce_init(&d, false);
    uint32_t rise = 0;
    uint32_t fall = 0;
    // A clean 500-sample pulse: rise accepted at sample k, fall accepted at sample 500 + k.
    for (uint32_t t = 0; t < 600u; t++) {
        const bool raw = t < 500u;
        if (emu_debounce_update(&d, raw)) {
            if (raw) {
                rise = t;
            } else {
                fall = t;
            }
        }
    }
    CHECK(fall - rise == 500u);
}

static emu_stepper_out_t run_ticks(emu_stepper_t *st, int32_t target, int count) {
    emu_stepper_out_t out = {0};
    for (int i = 0; i < count; i++) {
        out = emu_stepper_tick(st, target);
    }
    return out;
}

static void test_stepper_stays_disabled_and_silent_at_the_target(void) {
    emu_stepper_t st;
    emu_stepper_init(&st);
    for (int i = 0; i < 5000; i++) {
        const emu_stepper_out_t out = emu_stepper_tick(&st, 0);
        CHECK(!out.step);
        CHECK(!out.enable);
    }
}

static void test_stepper_enables_then_sets_direction_then_steps(void) {
    emu_stepper_t st;
    emu_stepper_init(&st);
    emu_stepper_out_t out = emu_stepper_tick(&st, 3);
    CHECK(out.enable);
    CHECK(!out.step);
    out = emu_stepper_tick(&st, 3);
    CHECK(out.enable);
    CHECK(out.dir_open);
    CHECK(!out.step);
    out = emu_stepper_tick(&st, 3);
    CHECK(out.step);
    CHECK(st.position == 1);
    out = emu_stepper_tick(&st, 3);
    CHECK(!out.step);  // the pulse lasts exactly one tick
}

static void test_stepper_reaches_the_target_without_exceeding_half_the_tick_rate(void) {
    emu_stepper_t st;
    emu_stepper_init(&st);
    int steps_seen = 0;
    int ticks_since_step = 100;
    for (int i = 0; i < 1000; i++) {
        const emu_stepper_out_t out = emu_stepper_tick(&st, 100);
        if (out.step) {
            CHECK(ticks_since_step >= 2);
            ticks_since_step = 0;
            steps_seen++;
        }
        ticks_since_step++;
    }
    CHECK(steps_seen == 100);
    CHECK(st.position == 100);
}

static void test_stepper_reverses_with_a_direction_gap_tick(void) {
    emu_stepper_t st;
    emu_stepper_init(&st);
    run_ticks(&st, 10, 100);
    CHECK(st.position == 10);
    emu_stepper_out_t out = emu_stepper_tick(&st, 5);  // step pulse of the last step is already low
    CHECK(!out.step);
    CHECK(!out.dir_open);
    out = emu_stepper_tick(&st, 5);
    CHECK(out.step);
    CHECK(!out.dir_open);
    CHECK(st.position == 9);
}

static void test_stepper_disables_after_the_hold_time(void) {
    emu_stepper_t st;
    emu_stepper_init(&st);
    emu_stepper_out_t out = run_ticks(&st, 4, 50);
    CHECK(st.position == 4);
    CHECK(out.enable);
    out = run_ticks(&st, 4, EMU_STEPPER_HOLD_TICKS + 2);
    CHECK(!out.enable);
}

int main(void) {
    RUN_TEST(test_travel_accepts_a_value_inside_the_bounds);
    RUN_TEST(test_travel_rejects_out_of_range_and_garbage);
    RUN_TEST(test_chop_accepts_hz_and_duty);
    RUN_TEST(test_chop_zero_means_steady_and_ignores_duty);
    RUN_TEST(test_chop_rejects_bad_hz_and_duty);
    RUN_TEST(test_chop_effective_period_and_on_time_follow_whole_milliseconds);
    RUN_TEST(test_glitch_reports_a_two_ms_blip);
    RUN_TEST(test_glitch_reports_chatter_as_one_episode_with_its_closed_time);
    RUN_TEST(test_glitch_ignores_a_clean_closure_with_bounce_on_both_edges);
    RUN_TEST(test_glitch_after_a_clean_closure_is_still_reported);
    RUN_TEST(test_glitch_waits_for_the_input_to_settle_before_reporting);
    RUN_TEST(test_reconfigure_is_allowed_only_when_closed_and_the_contact_is_released);
    RUN_TEST(test_reconfigure_is_refused_while_the_contact_is_held_raw_or_debounced);
    RUN_TEST(test_history_counts_closures_and_keeps_the_last_length);
    RUN_TEST(test_history_always_open_flag_is_sticky);
    RUN_TEST(test_steps_follow_position_permille);
    RUN_TEST(test_step_rate_at_the_shortest_travel_stays_below_2khz);
    RUN_TEST(test_debounce_ignores_a_glitch_shorter_than_the_window);
    RUN_TEST(test_debounce_reports_each_change_once);
    RUN_TEST(test_debounce_delays_both_edges_by_the_same_number_of_samples);
    RUN_TEST(test_stepper_stays_disabled_and_silent_at_the_target);
    RUN_TEST(test_stepper_enables_then_sets_direction_then_steps);
    RUN_TEST(test_stepper_reaches_the_target_without_exceeding_half_the_tick_rate);
    RUN_TEST(test_stepper_reverses_with_a_direction_gap_tick);
    RUN_TEST(test_stepper_disables_after_the_hold_time);
    return MINITEST_RESULT();
}
