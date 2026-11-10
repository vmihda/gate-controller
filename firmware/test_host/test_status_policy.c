// Host tests for the status LED pattern priority.
#include <stdbool.h>
#include <stdint.h>

#include "gate_config.h"
#include "minitest.h"
#include "status_policy.h"

// The "everything is fine" input; each test overrides only what it needs.
static status_inputs_t healthy(void) {
    status_inputs_t in = {.resetting = false, .command_error = false, .commissioned = true,
                          .wifi_connected = true, .started = true};
    return in;
}

static void test_ok_when_everything_works(void) {
    status_inputs_t in = healthy();
    CHECK(status_policy_select(&in) == LED_OK);
}

static void test_starting_wins_over_ok(void) {
    status_inputs_t in = healthy();
    in.started = false;
    CHECK(status_policy_select(&in) == LED_STARTING);
}

static void test_awaiting_commissioning_wins_over_starting(void) {
    status_inputs_t in = healthy();
    in.commissioned = false;
    in.wifi_connected = false;
    in.started = false;
    CHECK(status_policy_select(&in) == LED_AWAITING_COMMISSIONING);
    in.started = true;
    CHECK(status_policy_select(&in) == LED_AWAITING_COMMISSIONING);
}

static void test_no_wifi_wins_over_starting(void) {
    status_inputs_t in = healthy();
    in.wifi_connected = false;
    in.started = false;
    CHECK(status_policy_select(&in) == LED_NO_WIFI);
    in.started = true;
    CHECK(status_policy_select(&in) == LED_NO_WIFI);
}

static void test_error_wins_over_no_wifi(void) {
    status_inputs_t in = healthy();
    in.command_error = true;
    in.wifi_connected = false;
    in.started = false;
    CHECK(status_policy_select(&in) == LED_COMMAND_ERROR);
    in.commissioned = false;
    CHECK(status_policy_select(&in) == LED_COMMAND_ERROR);
}

static void test_resetting_wins_over_everything(void) {
    status_inputs_t in = {.resetting = true, .command_error = true, .commissioned = false,
                          .wifi_connected = false, .started = false};
    CHECK(status_policy_select(&in) == LED_RESETTING);
}

static void test_priority_order(void) {
    // Switch conditions on from the lowest priority to the highest; the pattern must follow.
    status_inputs_t in = healthy();
    CHECK(status_policy_select(&in) == LED_OK);
    in.started = false;
    CHECK(status_policy_select(&in) == LED_STARTING);
    in.commissioned = false;
    CHECK(status_policy_select(&in) == LED_AWAITING_COMMISSIONING);
    in.commissioned = true;
    in.wifi_connected = false;
    CHECK(status_policy_select(&in) == LED_NO_WIFI);
    in.command_error = true;
    CHECK(status_policy_select(&in) == LED_COMMAND_ERROR);
    in.resetting = true;
    CHECK(status_policy_select(&in) == LED_RESETTING);
}

static void test_wifi_state_ignored_before_commissioning(void) {
    status_inputs_t in = healthy();
    in.commissioned = false;
    CHECK(status_policy_select(&in) == LED_AWAITING_COMMISSIONING);
}

/* A common multiple of every pattern period, so each window starts at a period boundary. */
#define T0_MS 3000000u

/* Rising edges and lit time of a pattern, sampled every millisecond over [start_ms, start_ms + window_ms). */
typedef struct {
    uint32_t flashes;
    uint32_t lit_ms;
} led_activity_t;

static led_activity_t measure(led_pattern_t pattern, uint32_t start_ms, uint32_t window_ms) {
    led_activity_t a = {0u, 0u};
    bool was_on = status_policy_led_on(pattern, start_ms - 1u);
    for (uint32_t t = start_ms; t != start_ms + window_ms; t++) {
        const bool on = status_policy_led_on(pattern, t);
        if (on) {
            a.lit_ms++;
        }
        if (on && !was_on) {
            a.flashes++;
        }
        was_on = on;
    }
    return a;
}

static void test_starting_is_steady_on(void) {
    const led_activity_t a = measure(LED_STARTING, T0_MS, 5000u);
    CHECK(a.lit_ms == 5000u);
}

static void test_ok_is_one_short_flash_every_30_s(void) {
    const led_activity_t a = measure(LED_OK, T0_MS, 90000u);
    CHECK(a.flashes == 3u);
    CHECK(a.lit_ms <= 3u * 100u);
    CHECK(a.lit_ms > 0u);
}

static void test_awaiting_commissioning_blinks_once_per_second(void) {
    const led_activity_t a = measure(LED_AWAITING_COMMISSIONING, T0_MS, 10000u);
    CHECK(a.flashes == 10u);
}

static void test_no_wifi_is_two_short_flashes_every_2_s(void) {
    const led_activity_t a = measure(LED_NO_WIFI, T0_MS, 2000u);
    CHECK(a.flashes == 2u);
    CHECK(a.lit_ms <= 2u * 150u);
    const led_activity_t ten = measure(LED_NO_WIFI, T0_MS, 10000u);
    CHECK(ten.flashes == 10u);
}

static void test_command_error_blinks_faster_than_awaiting(void) {
    const led_activity_t error = measure(LED_COMMAND_ERROR, T0_MS, 10000u);
    const led_activity_t awaiting = measure(LED_AWAITING_COMMISSIONING, T0_MS, 10000u);
    CHECK(error.flashes >= 2u * awaiting.flashes);
}

static void test_resetting_blinks_faster_than_command_error(void) {
    const led_activity_t resetting = measure(LED_RESETTING, T0_MS, 10000u);
    const led_activity_t error = measure(LED_COMMAND_ERROR, T0_MS, 10000u);
    CHECK(resetting.flashes >= 2u * error.flashes);
}

static void test_patterns_keep_running_across_the_ms_wrap(void) {
    /* uint32_t milliseconds wrap after about 49.7 days; blinking must not stop there. */
    const led_activity_t a = measure(LED_COMMAND_ERROR, UINT32_MAX - 2000u, 4000u);
    CHECK(a.flashes >= 6u);
    const led_activity_t steady = measure(LED_STARTING, UINT32_MAX - 2000u, 4000u);
    CHECK(steady.lit_ms == 4000u);
}

/* Short BOOT press: the LED shows the state for GATE_LED_SHOW_MS. Times here are the 64-bit uptime. */
#define PRESS_MS ((uint64_t)T0_MS + 1000u)
#define NO_PRESS 0u

static const led_pattern_t ALL_PATTERNS[] = {LED_RESETTING, LED_COMMAND_ERROR, LED_NO_WIFI,
                                             LED_AWAITING_COMMISSIONING, LED_STARTING, LED_OK};

/* Milliseconds in [from_ms, to_ms) where status_policy_led_lit differs from the plain pattern. */
static uint32_t lit_differs_from_pattern(led_pattern_t pattern, uint64_t from_ms, uint64_t to_ms, uint64_t show_until_ms) {
    uint32_t differs = 0u;
    for (uint64_t t = from_ms; t < to_ms; t++) {
        if (status_policy_led_lit(pattern, t, show_until_ms) != status_policy_led_on(pattern, (uint32_t)t)) {
            differs++;
        }
    }
    return differs;
}

static uint32_t lit_ms(led_pattern_t pattern, uint64_t from_ms, uint64_t to_ms, uint64_t show_until_ms) {
    uint32_t lit = 0u;
    for (uint64_t t = from_ms; t < to_ms; t++) {
        if (status_policy_led_lit(pattern, t, show_until_ms)) {
            lit++;
        }
    }
    return lit;
}

static void test_without_a_press_the_led_follows_the_pattern(void) {
    for (unsigned i = 0u; i < sizeof ALL_PATTERNS / sizeof ALL_PATTERNS[0]; i++) {
        CHECK(lit_differs_from_pattern(ALL_PATTERNS[i], T0_MS, T0_MS + 60000u, NO_PRESS) == 0u);
    }
}

static void test_short_press_lights_a_healthy_device_steadily_for_3_s(void) {
    const uint64_t until = PRESS_MS + GATE_LED_SHOW_MS;
    CHECK(lit_ms(LED_OK, PRESS_MS, until, until) == GATE_LED_SHOW_MS);
    /* Then back to the short flash every 30 s. */
    CHECK(lit_differs_from_pattern(LED_OK, until, until + 60000u, until) == 0u);
}

static void test_short_press_leaves_the_other_patterns_as_they_are(void) {
    /* They already show the state: the press must not hide an error, a lost Wi-Fi or a reset. */
    const uint64_t until = PRESS_MS + GATE_LED_SHOW_MS;
    for (unsigned i = 0u; i < sizeof ALL_PATTERNS / sizeof ALL_PATTERNS[0]; i++) {
        if (ALL_PATTERNS[i] != LED_OK) {
            CHECK(lit_differs_from_pattern(ALL_PATTERNS[i], PRESS_MS, until, until) == 0u);
        }
    }
}

static void test_show_window_crosses_the_32_bit_ms_wrap(void) {
    /* After about 49.7 days of uptime a press must still show for exactly 3 s, not end at once or stay on. */
    const uint64_t press = (uint64_t)UINT32_MAX - 1000u;
    const uint64_t until = press + GATE_LED_SHOW_MS;
    CHECK(lit_ms(LED_OK, press, until, until) == GATE_LED_SHOW_MS);
    CHECK(lit_differs_from_pattern(LED_OK, until, until + 60000u, until) == 0u);
}

static void test_old_press_does_not_come_back_after_the_32_bit_wrap(void) {
    /* A press 2^32 ms ago must not light the LED again when the 32-bit clock comes round. */
    const uint64_t until = PRESS_MS + GATE_LED_SHOW_MS;
    const uint64_t later = PRESS_MS + ((uint64_t)1u << 32);
    CHECK(lit_differs_from_pattern(LED_OK, later, later + GATE_LED_SHOW_MS, until) == 0u);
}

// Final review M-2: "started" latches on the first recognized state. The decoder reports UNKNOWN at the
// start and end of every gate motion; that must not bring back the steady "starting" LED.
static void test_started_latches_on_the_first_recognized_state(void) {
    bool started = false;
    started = status_policy_started(started, false);
    CHECK(!started);
    started = status_policy_started(started, true);
    CHECK(started);
    started = status_policy_started(started, false);
    CHECK(started);
}

int main(void) {
    RUN_TEST(test_ok_when_everything_works);
    RUN_TEST(test_starting_wins_over_ok);
    RUN_TEST(test_awaiting_commissioning_wins_over_starting);
    RUN_TEST(test_no_wifi_wins_over_starting);
    RUN_TEST(test_error_wins_over_no_wifi);
    RUN_TEST(test_resetting_wins_over_everything);
    RUN_TEST(test_priority_order);
    RUN_TEST(test_wifi_state_ignored_before_commissioning);
    RUN_TEST(test_starting_is_steady_on);
    RUN_TEST(test_ok_is_one_short_flash_every_30_s);
    RUN_TEST(test_awaiting_commissioning_blinks_once_per_second);
    RUN_TEST(test_no_wifi_is_two_short_flashes_every_2_s);
    RUN_TEST(test_command_error_blinks_faster_than_awaiting);
    RUN_TEST(test_resetting_blinks_faster_than_command_error);
    RUN_TEST(test_patterns_keep_running_across_the_ms_wrap);
    RUN_TEST(test_without_a_press_the_led_follows_the_pattern);
    RUN_TEST(test_short_press_lights_a_healthy_device_steadily_for_3_s);
    RUN_TEST(test_short_press_leaves_the_other_patterns_as_they_are);
    RUN_TEST(test_show_window_crosses_the_32_bit_ms_wrap);
    RUN_TEST(test_old_press_does_not_come_back_after_the_32_bit_wrap);
    RUN_TEST(test_started_latches_on_the_first_recognized_state);
    return MINITEST_RESULT();
}
