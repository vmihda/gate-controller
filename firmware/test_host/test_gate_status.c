// Host tests for gate_status: when the gate task calls its listener.
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "gate_controller.h"
#include "gate_status.h"
#include "minitest.h"

static bool accept_pulse(void *ctx)
{
    (void)ctx;
    return true;
}

static const gate_status_t closed_idle = {.state = GATE_STATE_CLOSED, .busy = false, .error = GATE_ERR_NONE};

static void test_unchanged_status_without_a_command_is_not_published(void) {
    const gate_status_t now = closed_idle;
    CHECK(!gate_status_publish_due(&closed_idle, &now, false));
}

static void test_every_field_change_is_published(void) {
    gate_status_t now = closed_idle;
    now.state = GATE_STATE_OPENING;
    CHECK(gate_status_publish_due(&closed_idle, &now, false));
    now = closed_idle;
    now.busy = true;
    CHECK(gate_status_publish_due(&closed_idle, &now, false));
    now = closed_idle;
    now.error = GATE_ERR_NO_CONFIRM;
    CHECK(gate_status_publish_due(&closed_idle, &now, false));
}

static void test_a_taken_command_is_published_even_if_nothing_changed(void) {
    const gate_status_t now = closed_idle;
    CHECK(gate_status_publish_due(&closed_idle, &now, true));
}

// The case behind the rule: Home sends "close" to a closed gate. The controller finishes the command in the
// same step with no error, so the status the gate task sees afterwards equals the one it published before.
// Only the taken command makes it publish, and Home then gets "stopped, closed" instead of "closing" forever.
static void test_close_on_a_closed_gate_leaves_the_status_unchanged_but_is_published(void) {
    gate_controller_t c;
    gate_controller_init(&c, 0u, accept_pulse, NULL);
    (void)gate_controller_step(&c, GATE_STATE_CLOSED, GATE_MOTION_NONE, false, 10u);
    const gate_status_t before = {GATE_STATE_CLOSED, gate_controller_busy(&c), gate_controller_last_error(&c)};

    gate_controller_command(&c, GATE_CMD_CLOSE, 20u);
    CHECK(!gate_controller_step(&c, GATE_STATE_CLOSED, GATE_MOTION_NONE, false, 20u));
    const gate_status_t after = {GATE_STATE_CLOSED, gate_controller_busy(&c), gate_controller_last_error(&c)};

    CHECK(!gate_status_publish_due(&before, &after, false));
    CHECK(gate_status_publish_due(&before, &after, true));
}

static void test_open_on_an_open_gate_leaves_the_status_unchanged_but_is_published(void) {
    gate_controller_t c;
    gate_controller_init(&c, 0u, accept_pulse, NULL);
    (void)gate_controller_step(&c, GATE_STATE_NOT_CLOSED, GATE_MOTION_NONE, false, 10u);
    const gate_status_t before = {GATE_STATE_NOT_CLOSED, gate_controller_busy(&c), gate_controller_last_error(&c)};

    gate_controller_command(&c, GATE_CMD_OPEN, 20u);
    CHECK(!gate_controller_step(&c, GATE_STATE_NOT_CLOSED, GATE_MOTION_NONE, false, 20u));
    const gate_status_t after = {GATE_STATE_NOT_CLOSED, gate_controller_busy(&c), gate_controller_last_error(&c)};

    CHECK(!gate_status_publish_due(&before, &after, false));
    CHECK(gate_status_publish_due(&before, &after, true));
}

// The names go to the USB log; GATE_SIM prints them on the console, where the Wokwi scenarios match them.
static void test_every_state_has_its_name(void) {
    CHECK(strcmp(gate_state_name(GATE_STATE_UNKNOWN), "UNKNOWN") == 0);
    CHECK(strcmp(gate_state_name(GATE_STATE_CLOSED), "CLOSED") == 0);
    CHECK(strcmp(gate_state_name(GATE_STATE_NOT_CLOSED), "NOT_CLOSED") == 0);
    CHECK(strcmp(gate_state_name(GATE_STATE_OPENING), "OPENING") == 0);
    CHECK(strcmp(gate_state_name(GATE_STATE_CLOSING), "CLOSING") == 0);
}

static void test_every_error_has_its_name(void) {
    CHECK(strcmp(gate_error_name(GATE_ERR_NONE), "NONE") == 0);
    CHECK(strcmp(gate_error_name(GATE_ERR_UNKNOWN_STATE), "UNKNOWN_STATE") == 0);
    CHECK(strcmp(gate_error_name(GATE_ERR_NO_CONFIRM), "NO_CONFIRM") == 0);
    CHECK(strcmp(gate_error_name(GATE_ERR_WRONG_DIRECTION), "WRONG_DIRECTION") == 0);
    CHECK(strcmp(gate_error_name(GATE_ERR_OBSTACLE), "OBSTACLE") == 0);
    CHECK(strcmp(gate_error_name(GATE_ERR_UNEXPECTED_STOP), "UNEXPECTED_STOP") == 0);
    CHECK(strcmp(gate_error_name(GATE_ERR_TRAVEL_TIMEOUT), "TRAVEL_TIMEOUT") == 0);
}

int main(void) {
    RUN_TEST(test_unchanged_status_without_a_command_is_not_published);
    RUN_TEST(test_every_field_change_is_published);
    RUN_TEST(test_a_taken_command_is_published_even_if_nothing_changed);
    RUN_TEST(test_close_on_a_closed_gate_leaves_the_status_unchanged_but_is_published);
    RUN_TEST(test_open_on_an_open_gate_leaves_the_status_unchanged_but_is_published);
    RUN_TEST(test_every_state_has_its_name);
    RUN_TEST(test_every_error_has_its_name);
    return MINITEST_RESULT();
}
