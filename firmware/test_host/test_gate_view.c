// Host tests for gate_view: what Home is told about the gate, from the gate task's status.
#include <stdbool.h>

#include "gate_view.h"
#include "minitest.h"

static gate_view_t fresh(void) {
    gate_view_t v;
    gate_view_init(&v);
    return v;
}

// A view that has seen the gate stationary in `state` with no command running.
static gate_view_t settled(gate_state_t state) {
    gate_view_t v = fresh();
    gate_view_update(&v, state, false, GATE_ERR_NONE);
    return v;
}

static void test_position_is_unknown_until_the_first_recognized_state(void) {
    gate_view_t v = fresh();
    CHECK(v.position == GATE_POSITION_UNKNOWN);
    CHECK(v.motion == GATE_MOTION_NONE);
    gate_view_update(&v, GATE_STATE_UNKNOWN, false, GATE_ERR_NONE);
    CHECK(v.position == GATE_POSITION_UNKNOWN);
}

static void test_closed_is_closed(void) {
    const gate_view_t v = settled(GATE_STATE_CLOSED);
    CHECK(v.position == GATE_POSITION_CLOSED);
    CHECK(v.motion == GATE_MOTION_NONE);
}

static void test_not_closed_is_open(void) {
    const gate_view_t v = settled(GATE_STATE_NOT_CLOSED);
    CHECK(v.position == GATE_POSITION_OPEN);
    CHECK(v.motion == GATE_MOTION_NONE);
}

static void test_motion_keeps_the_last_stationary_position(void) {
    gate_view_t v = settled(GATE_STATE_CLOSED);
    gate_view_update(&v, GATE_STATE_OPENING, false, GATE_ERR_NONE);
    CHECK(v.position == GATE_POSITION_CLOSED);
    CHECK(v.motion == GATE_MOTION_OPENING);

    v = settled(GATE_STATE_NOT_CLOSED);
    gate_view_update(&v, GATE_STATE_CLOSING, false, GATE_ERR_NONE);
    CHECK(v.position == GATE_POSITION_OPEN);
    CHECK(v.motion == GATE_MOTION_CLOSING);
}

static void test_reaching_a_stationary_state_ends_the_motion(void) {
    gate_view_t v = settled(GATE_STATE_CLOSED);
    gate_view_update(&v, GATE_STATE_OPENING, false, GATE_ERR_NONE);
    gate_view_update(&v, GATE_STATE_NOT_CLOSED, false, GATE_ERR_NONE);
    CHECK(v.position == GATE_POSITION_OPEN);
    CHECK(v.motion == GATE_MOTION_NONE);
}

static void test_unknown_keeps_the_last_stationary_state(void) {
    gate_view_t v = settled(GATE_STATE_NOT_CLOSED);
    gate_view_update(&v, GATE_STATE_UNKNOWN, false, GATE_ERR_NONE);
    CHECK(v.position == GATE_POSITION_OPEN);
    CHECK(v.motion == GATE_MOTION_NONE);
}

static void test_unknown_inside_a_blinking_run_keeps_the_motion(void) {
    // The decoder reports UNKNOWN at the start and the end of every blinking run; Home must not flap.
    gate_view_t v = settled(GATE_STATE_CLOSED);
    gate_view_update(&v, GATE_STATE_UNKNOWN, false, GATE_ERR_NONE);
    gate_view_update(&v, GATE_STATE_OPENING, false, GATE_ERR_NONE);
    gate_view_update(&v, GATE_STATE_UNKNOWN, false, GATE_ERR_NONE);
    CHECK(v.position == GATE_POSITION_CLOSED);
    CHECK(v.motion == GATE_MOTION_OPENING);
}

// Every published status is folded in, in order: a CLOSED that UNKNOWN follows before Home is written still
// moves an open view to closed (matter_app folds at publish time for this reason).
static void test_a_state_followed_by_unknown_still_counts(void) {
    gate_view_t v = settled(GATE_STATE_NOT_CLOSED);
    gate_view_update(&v, GATE_STATE_CLOSING, false, GATE_ERR_NONE);
    gate_view_update(&v, GATE_STATE_CLOSED, false, GATE_ERR_NONE);
    gate_view_update(&v, GATE_STATE_UNKNOWN, false, GATE_ERR_NONE);
    CHECK(v.position == GATE_POSITION_CLOSED);
    CHECK(v.motion == GATE_MOTION_NONE);
}

static void test_busy_and_error_follow_the_gate_task(void) {
    gate_view_t v = settled(GATE_STATE_CLOSED);
    gate_view_update(&v, GATE_STATE_CLOSED, true, GATE_ERR_NONE);
    CHECK(v.busy);
    gate_view_update(&v, GATE_STATE_UNKNOWN, false, GATE_ERR_UNKNOWN_STATE);
    CHECK(!v.busy);
    CHECK(v.error == GATE_ERR_UNKNOWN_STATE);
    gate_view_update(&v, GATE_STATE_CLOSED, true, GATE_ERR_NONE);
    CHECK(v.error == GATE_ERR_NONE);
}

static void test_active_while_moving_or_running_a_command(void) {
    gate_view_t v = settled(GATE_STATE_CLOSED);
    CHECK(!gate_view_active(&v));
    gate_view_update(&v, GATE_STATE_CLOSED, true, GATE_ERR_NONE);  // command waits for its pulse
    CHECK(gate_view_active(&v));
    gate_view_update(&v, GATE_STATE_OPENING, false, GATE_ERR_NONE);  // wall button
    CHECK(gate_view_active(&v));
    gate_view_update(&v, GATE_STATE_NOT_CLOSED, false, GATE_ERR_NONE);
    CHECK(!gate_view_active(&v));
}

static void test_target_is_the_position_when_idle(void) {
    gate_position_t target = GATE_POSITION_UNKNOWN;
    gate_view_t v = settled(GATE_STATE_NOT_CLOSED);
    CHECK(gate_view_target(&v, &target));
    CHECK(target == GATE_POSITION_OPEN);

    v = fresh();
    CHECK(gate_view_target(&v, &target));
    CHECK(target == GATE_POSITION_UNKNOWN);
}

static void test_target_follows_the_motion(void) {
    gate_position_t target = GATE_POSITION_UNKNOWN;
    gate_view_t v = settled(GATE_STATE_CLOSED);
    gate_view_update(&v, GATE_STATE_OPENING, false, GATE_ERR_NONE);
    CHECK(gate_view_target(&v, &target));
    CHECK(target == GATE_POSITION_OPEN);

    // Moving against a command: Home is told where the gate really goes.
    v = settled(GATE_STATE_NOT_CLOSED);
    gate_view_update(&v, GATE_STATE_CLOSING, true, GATE_ERR_NONE);
    CHECK(gate_view_target(&v, &target));
    CHECK(target == GATE_POSITION_CLOSED);
}

static void test_target_is_left_to_the_command_before_the_gate_moves(void) {
    gate_position_t target = GATE_POSITION_UNKNOWN;
    gate_view_t v = settled(GATE_STATE_CLOSED);
    gate_view_update(&v, GATE_STATE_CLOSED, true, GATE_ERR_NONE);
    CHECK(!gate_view_target(&v, &target));
}

static void test_target_returns_to_the_position_after_a_failed_command(void) {
    gate_position_t target = GATE_POSITION_UNKNOWN;
    gate_view_t v = settled(GATE_STATE_CLOSED);
    gate_view_update(&v, GATE_STATE_CLOSED, true, GATE_ERR_NONE);
    gate_view_update(&v, GATE_STATE_CLOSED, false, GATE_ERR_NO_CONFIRM);
    CHECK(gate_view_target(&v, &target));
    CHECK(target == GATE_POSITION_CLOSED);
}

// Final review M-1: a two-state display (the Door Lock candidate) shows "shut" only when the gate is
// closed and still; any motion, including the opening that starts from closed, shows it open.
static void test_shut_only_when_closed_and_still(void) {
    bool shut = true;
    gate_view_t v = fresh();
    CHECK(!gate_view_shut(&v, &shut));  // nothing known yet: leave the display as it is
    v = settled(GATE_STATE_CLOSED);
    CHECK(gate_view_shut(&v, &shut) && shut);
    gate_view_update(&v, GATE_STATE_OPENING, false, GATE_ERR_NONE);
    CHECK(gate_view_shut(&v, &shut) && !shut);
    v = settled(GATE_STATE_NOT_CLOSED);
    CHECK(gate_view_shut(&v, &shut) && !shut);
    gate_view_update(&v, GATE_STATE_CLOSING, false, GATE_ERR_NONE);
    CHECK(gate_view_shut(&v, &shut) && !shut);
    gate_view_update(&v, GATE_STATE_CLOSED, false, GATE_ERR_NONE);
    CHECK(gate_view_shut(&v, &shut) && shut);
}

int main(void) {
    RUN_TEST(test_position_is_unknown_until_the_first_recognized_state);
    RUN_TEST(test_closed_is_closed);
    RUN_TEST(test_not_closed_is_open);
    RUN_TEST(test_motion_keeps_the_last_stationary_position);
    RUN_TEST(test_reaching_a_stationary_state_ends_the_motion);
    RUN_TEST(test_unknown_keeps_the_last_stationary_state);
    RUN_TEST(test_unknown_inside_a_blinking_run_keeps_the_motion);
    RUN_TEST(test_a_state_followed_by_unknown_still_counts);
    RUN_TEST(test_busy_and_error_follow_the_gate_task);
    RUN_TEST(test_active_while_moving_or_running_a_command);
    RUN_TEST(test_target_is_the_position_when_idle);
    RUN_TEST(test_target_follows_the_motion);
    RUN_TEST(test_target_is_left_to_the_command_before_the_gate_moves);
    RUN_TEST(test_target_returns_to_the_position_after_a_failed_command);
    RUN_TEST(test_shut_only_when_closed_and_still);
    return MINITEST_RESULT();
}
