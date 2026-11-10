// Host tests for the BOOT button press classification. The button is sampled every POLL_MS, as on the device.
// Every press starts near the uint32_t millisecond wrap, so the hold time is always measured across it.
#include <stdbool.h>
#include <stdint.h>

#include "button_press.h"
#include "gate_config.h"
#include "minitest.h"

#define POLL_MS 20u
#define T0_MS (UINT32_MAX - 3000u)

typedef struct {
    button_press_t button;
    uint32_t now_ms;
    uint32_t start_ms;
    unsigned shorts;
    unsigned longs;
    uint32_t long_at_ms;  // elapsed since start_ms when the long press was reported
} rig_t;

// A cold rig: the button state is as it is at power-up, with no sample taken yet.
static void rig_init_cold(rig_t *r)
{
    button_press_init(&r->button);
    r->now_ms = T0_MS;
    r->start_ms = T0_MS;
    r->shorts = 0u;
    r->longs = 0u;
    r->long_at_ms = 0u;
}

// Samples the button every POLL_MS for duration_ms, then once more at the end.
static void rig_hold(rig_t *r, bool pressed, uint32_t duration_ms)
{
    for (uint32_t done = 0u; done <= duration_ms; done += POLL_MS) {
        switch (button_press_sample(&r->button, pressed, r->now_ms)) {
        case BUTTON_EVENT_SHORT:
            r->shorts++;
            break;
        case BUTTON_EVENT_LONG:
            r->longs++;
            r->long_at_ms = r->now_ms - r->start_ms;
            break;
        case BUTTON_EVENT_NONE:
            break;
        }
        if (done < duration_ms) {
            r->now_ms += POLL_MS;
        }
    }
}

// A rig whose button has been seen released for longer than the debounce time, so presses can arm.
static void rig_init(rig_t *r)
{
    rig_init_cold(r);
    rig_hold(r, false, 2u * GATE_BUTTON_DEBOUNCE_MS);
    r->now_ms += POLL_MS;
}

// A press of press_ms between two released samples.
static void rig_press(rig_t *r, uint32_t press_ms)
{
    rig_hold(r, false, POLL_MS);
    r->now_ms += POLL_MS;
    r->start_ms = r->now_ms;
    rig_hold(r, true, press_ms);
    r->now_ms += POLL_MS;
    rig_hold(r, false, POLL_MS);
}

static void test_released_button_reports_nothing(void)
{
    rig_t r;
    rig_init(&r);
    rig_hold(&r, false, 10000u);
    CHECK(r.shorts == 0u);
    CHECK(r.longs == 0u);
}

static void test_short_press_is_reported_once_at_release(void)
{
    rig_t r;
    rig_init(&r);
    r.start_ms = r.now_ms;
    rig_hold(&r, true, 200u);
    CHECK(r.shorts == 0u);  // nothing while it is still held
    r.now_ms += POLL_MS;
    rig_hold(&r, false, 1000u);
    CHECK(r.shorts == 1u);
    CHECK(r.longs == 0u);
}

static void test_bounce_shorter_than_debounce_is_ignored(void)
{
    rig_t r;
    rig_init(&r);
    rig_press(&r, GATE_BUTTON_DEBOUNCE_MS - POLL_MS);
    CHECK(r.shorts == 0u);
    CHECK(r.longs == 0u);
}

static void test_press_of_debounce_length_counts(void)
{
    rig_t r;
    rig_init(&r);
    rig_press(&r, GATE_BUTTON_DEBOUNCE_MS);
    CHECK(r.shorts == 1u);
}

static void test_long_press_is_reported_once_while_held(void)
{
    rig_t r;
    rig_init(&r);
    r.start_ms = r.now_ms;
    rig_hold(&r, true, GATE_BUTTON_LONG_PRESS_MS + 3000u);
    CHECK(r.longs == 1u);
    CHECK(r.long_at_ms >= GATE_BUTTON_LONG_PRESS_MS);
    CHECK(r.long_at_ms < GATE_BUTTON_LONG_PRESS_MS + POLL_MS);
    r.now_ms += POLL_MS;
    rig_hold(&r, false, 1000u);
    CHECK(r.shorts == 0u);  // the release after a long press is not also a short press
    CHECK(r.longs == 1u);
}

static void test_press_just_short_of_long_is_a_short_press(void)
{
    rig_t r;
    rig_init(&r);
    rig_press(&r, GATE_BUTTON_LONG_PRESS_MS - POLL_MS);
    CHECK(r.shorts == 1u);
    CHECK(r.longs == 0u);
}

static void test_next_press_after_a_long_press_counts_again(void)
{
    rig_t r;
    rig_init(&r);
    rig_press(&r, GATE_BUTTON_LONG_PRESS_MS + 1000u);
    rig_press(&r, 200u);
    CHECK(r.longs == 1u);
    CHECK(r.shorts == 1u);
}

static void test_two_short_presses_are_two_events(void)
{
    rig_t r;
    rig_init(&r);
    rig_press(&r, 200u);
    rig_press(&r, 300u);
    CHECK(r.shorts == 2u);
    CHECK(r.longs == 0u);
}

static void test_button_held_from_the_first_sample_never_gives_long(void)
{
    rig_t r;
    rig_init_cold(&r);
    rig_hold(&r, true, 10000u);
    CHECK(r.longs == 0u);
    CHECK(r.shorts == 0u);
    r.now_ms += POLL_MS;
    rig_hold(&r, false, 1000u);
    CHECK(r.shorts == 0u);  // releasing a stuck button is not a press either
    CHECK(r.longs == 0u);
}

static void test_press_after_a_release_of_a_stuck_button_gives_long(void)
{
    rig_t r;
    rig_init_cold(&r);
    rig_hold(&r, true, 10000u);
    r.now_ms += POLL_MS;
    rig_hold(&r, false, 2u * GATE_BUTTON_DEBOUNCE_MS);
    r.now_ms += POLL_MS;
    r.start_ms = r.now_ms;
    rig_hold(&r, true, GATE_BUTTON_LONG_PRESS_MS + POLL_MS);
    CHECK(r.longs == 1u);
}

static void test_release_shorter_than_debounce_does_not_arm(void)
{
    rig_t r;
    rig_init_cold(&r);
    rig_hold(&r, true, 1000u);
    r.now_ms += POLL_MS;
    rig_hold(&r, false, GATE_BUTTON_DEBOUNCE_MS - 2u * POLL_MS);  // contact bounce of a stuck button
    r.now_ms += POLL_MS;
    rig_hold(&r, true, GATE_BUTTON_LONG_PRESS_MS + 1000u);
    CHECK(r.longs == 0u);
}

int main(void)
{
    RUN_TEST(test_button_held_from_the_first_sample_never_gives_long);
    RUN_TEST(test_press_after_a_release_of_a_stuck_button_gives_long);
    RUN_TEST(test_release_shorter_than_debounce_does_not_arm);
    RUN_TEST(test_released_button_reports_nothing);
    RUN_TEST(test_short_press_is_reported_once_at_release);
    RUN_TEST(test_bounce_shorter_than_debounce_is_ignored);
    RUN_TEST(test_press_of_debounce_length_counts);
    RUN_TEST(test_long_press_is_reported_once_while_held);
    RUN_TEST(test_press_just_short_of_long_is_a_short_press);
    RUN_TEST(test_next_press_after_a_long_press_counts_again);
    RUN_TEST(test_two_short_presses_are_two_events);
    return MINITEST_RESULT();
}
