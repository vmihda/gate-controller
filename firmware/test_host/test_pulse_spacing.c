// Host tests for the relay pulse spacing rule that relay_out applies with its own clock.
#include <stddef.h>
#include <stdint.h>

#include "gate_config.h"
#include "minitest.h"
#include "pulse_spacing.h"

#define WRAP_MARGIN_MS 3000u
#define PREEMPTION_MS 200u

static uint32_t g_t0;

static void test_first_pulse_is_allowed(void)
{
    pulse_spacing_t s;
    pulse_spacing_init(&s);
    CHECK(pulse_spacing_allows(&s, g_t0));
    CHECK(pulse_spacing_allows(&s, g_t0 + 123u));
}

static void test_next_pulse_waits_for_the_spacing(void)
{
    pulse_spacing_t s;
    pulse_spacing_init(&s);
    pulse_spacing_started(&s, g_t0);
    CHECK(!pulse_spacing_allows(&s, g_t0));
    CHECK(!pulse_spacing_allows(&s, g_t0 + GATE_PULSE_MS));
    CHECK(!pulse_spacing_allows(&s, g_t0 + GATE_PULSE_SPACING_MS - 1u));
    CHECK(pulse_spacing_allows(&s, g_t0 + GATE_PULSE_SPACING_MS));
    CHECK(pulse_spacing_allows(&s, g_t0 + 10u * GATE_PULSE_SPACING_MS));
}

// Review 6-7 item 1: the controller stamps a pulse at t, but the task is preempted and the pulse really
// starts at t + 200 ms. The controller's next pulse at t + 1500 would leave only 800 ms low; relay_out,
// which stamps the real start, refuses it and accepts it 1500 ms after the real start.
static void test_preempted_start_moves_the_next_slot(void)
{
    const uint32_t controller_ms = g_t0;
    const uint32_t real_start_ms = controller_ms + PREEMPTION_MS;
    pulse_spacing_t s;
    pulse_spacing_init(&s);
    CHECK(pulse_spacing_allows(&s, real_start_ms));
    pulse_spacing_started(&s, real_start_ms);
    CHECK(!pulse_spacing_allows(&s, controller_ms + GATE_PULSE_SPACING_MS));
    CHECK(pulse_spacing_allows(&s, real_start_ms + GATE_PULSE_SPACING_MS));
}

// The latest start counts: an accepted pulse moves the slot on.
static void test_each_start_moves_the_slot(void)
{
    pulse_spacing_t s;
    pulse_spacing_init(&s);
    pulse_spacing_started(&s, g_t0);
    pulse_spacing_started(&s, g_t0 + GATE_PULSE_SPACING_MS);
    CHECK(!pulse_spacing_allows(&s, g_t0 + 2u * GATE_PULSE_SPACING_MS - 1u));
    CHECK(pulse_spacing_allows(&s, g_t0 + 2u * GATE_PULSE_SPACING_MS));
}

// Starts on both sides of the 32-bit wrap of the millisecond clock.
static void test_spacing_across_the_wrap(void)
{
    const uint32_t before_wrap[] = {UINT32_MAX, UINT32_MAX - GATE_PULSE_MS, UINT32_MAX - GATE_PULSE_SPACING_MS + 1u};
    for (size_t i = 0; i < sizeof before_wrap / sizeof before_wrap[0]; i++) {
        pulse_spacing_t s;
        pulse_spacing_init(&s);
        pulse_spacing_started(&s, before_wrap[i]);
        CHECK(!pulse_spacing_allows(&s, before_wrap[i] + GATE_PULSE_SPACING_MS - 1u));
        CHECK(pulse_spacing_allows(&s, before_wrap[i] + GATE_PULSE_SPACING_MS));
    }
}

int main(void)
{
    const uint32_t starts[] = {0u, UINT32_MAX - WRAP_MARGIN_MS};
    for (size_t i = 0; i < sizeof starts / sizeof starts[0]; i++) {
        g_t0 = starts[i];
        RUN_TEST(test_first_pulse_is_allowed);
        RUN_TEST(test_next_pulse_waits_for_the_spacing);
        RUN_TEST(test_preempted_start_moves_the_next_slot);
        RUN_TEST(test_each_start_moves_the_slot);
        RUN_TEST(test_spacing_across_the_wrap);
    }
    return MINITEST_RESULT();
}
