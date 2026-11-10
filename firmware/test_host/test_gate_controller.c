// Host tests for the command controller. The test feeds decoder states in 10 ms steps and records
// the moments when step() asks for a relay pulse.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "gate_config.h"
#include "gate_controller.h"
#include "minitest.h"

#define TICK_MS 10u
#define WRAP_MARGIN_MS 3000u
#define PULSE_LOG_SIZE 64u
#define PULSE_SPACING_MS (GATE_PULSE_SPACING_MS + GATE_PULSE_SPACING_MARGIN_MS)  // controller distance between pulse starts
#define CONFIRM_LIMIT_MS (GATE_PULSE_MS + GATE_CONFIRM_TIMEOUT_MS)  // counted from the pulse start
#define LEAD_MS 100u         // quiet time before a command
#define REACTION_MS 300u     // decoder still reports the old state after a pulse
#define RECOGNIZE_MS 1200u   // UNKNOWN before a blink is classified
#define SETTLE_MS 500u       // UNKNOWN before a stop is confirmed
#define TRAVEL_MS 8000u
#define LONG_MS 5000u
#define NO_COMMAND_RUN_MS (10u * 60u * 1000u)
#define FUZZ_RUN_MS (30u * 60u * 1000u)
#define FUZZ_SEGMENT_MAX_MS 3000u
#define FUZZ_COMMAND_ODDS 4u
#define FUZZ_SEED 0x2545F491u
// A motion state confirms itself once it has lasted longer than the decoder can hold one reading without
// a new edge (GATE_STEADY_AFTER_MS); this is the first tick after that.
#define CORROBORATED_AFTER_MS ((GATE_STEADY_AFTER_MS / TICK_MS + 1u) * TICK_MS)
// A motion state shown this long gets the stop pulse on its last tick.
#define CONFIRM_RUN_MS (CORROBORATED_AFTER_MS + TICK_MS)

static uint32_t g_t0;

typedef struct {
    gate_controller_t ctl;
    uint32_t start_ms;
    uint32_t now_ms;
    unsigned pulses;      // total since rig_init
    unsigned cmd_pulses;  // since the last command
    uint32_t pulse_at[PULSE_LOG_SIZE];  // elapsed since start
    bool any_pulse;
    uint32_t last_pulse_ms;
    bool relay_refuses;   // the relay refuses every pulse while set
    bool edges_stale;     // a motion state is fed without a recent FLASH edge (a stopped gate, held reading)
    unsigned refusals;
} rig_t;

static bool rig_relay(void *ctx)
{
    rig_t *r = ctx;
    if (r->relay_refuses) {
        r->refusals++;
        return false;
    }
    return true;
}

static void rig_init(rig_t *r)
{
    memset(r, 0, sizeof *r);
    r->start_ms = g_t0;
    r->now_ms = g_t0;
    gate_controller_init(&r->ctl, g_t0, rig_relay, r);
}

static uint32_t rig_elapsed(const rig_t *r)
{
    return (uint32_t)(r->now_ms - r->start_ms);
}

static void rig_command(rig_t *r, gate_cmd_t cmd)
{
    gate_controller_command(&r->ctl, cmd, r->now_ms);
    r->cmd_pulses = 0;
}

// One step every TICK_MS for duration_ms, starting at the current time. Every pulse is checked
// against the invariants that hold in any scenario.
static void rig_run_lm(rig_t *r, gate_state_t state, gate_motion_t last_motion, uint32_t duration_ms)
{
    for (uint32_t done = 0; done < duration_ms; done += TICK_MS) {
        const bool was_busy = gate_controller_busy(&r->ctl);
        // A motion state normally comes from a blinking gate: fresh unless the test says otherwise.
        const bool fresh = (state == GATE_STATE_OPENING || state == GATE_STATE_CLOSING) && !r->edges_stale;
        if (gate_controller_step(&r->ctl, state, last_motion, fresh, r->now_ms)) {
            CHECK(was_busy);
            CHECK(state != GATE_STATE_UNKNOWN);
            if (r->any_pulse) {
                CHECK((uint32_t)(r->now_ms - r->last_pulse_ms) >= PULSE_SPACING_MS);
            }
            r->cmd_pulses++;
            CHECK(r->cmd_pulses <= GATE_MAX_PULSES);
            if (r->pulses < PULSE_LOG_SIZE) {
                r->pulse_at[r->pulses] = rig_elapsed(r);
            }
            r->pulses++;
            r->any_pulse = true;
            r->last_pulse_ms = r->now_ms;
        }
        r->now_ms += TICK_MS;
    }
}

static void rig_run(rig_t *r, gate_state_t state, uint32_t duration_ms)
{
    rig_run_lm(r, state, GATE_MOTION_NONE, duration_ms);
}

// The gate has been moving for a while before the test starts: the motion is confirmed. Elapsed time
// counts from the end of that history.
static void rig_init_moving(rig_t *r, gate_state_t motion)
{
    rig_init(r);
    rig_run(r, motion, CORROBORATED_AFTER_MS);
    r->start_ms = r->now_ms;
}

static void check_done(const rig_t *r, gate_error_t expected)
{
    CHECK(!gate_controller_busy(&r->ctl));
    CHECK(gate_controller_last_error(&r->ctl) == expected);
}

// Small deterministic generator for the randomized tests.
static uint32_t next_random(uint32_t *seed)
{
    uint32_t x = *seed;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *seed = x;
    return x;
}

// A random decoder state and last motion for a random time.
static void run_random_segment(rig_t *r, uint32_t *seed)
{
    const gate_state_t state = (gate_state_t)(next_random(seed) % (GATE_STATE_CLOSING + 1u));
    const gate_motion_t motion = (gate_motion_t)(next_random(seed) % (GATE_MOTION_CLOSING + 1u));
    const uint32_t duration = TICK_MS * (1u + next_random(seed) % (FUZZ_SEGMENT_MAX_MS / TICK_MS));
    rig_run_lm(r, state, motion, duration);
}

static void test_open_from_closed(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSED, REACTION_MS);
    CHECK(r.pulses == 1u);
    CHECK(r.pulse_at[0] == LEAD_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_OPENING, TRAVEL_MS);
    CHECK(gate_controller_busy(&r.ctl));
    rig_run(&r, GATE_STATE_NOT_CLOSED, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    rig_run(&r, GATE_STATE_NOT_CLOSED, LONG_MS);
    CHECK(r.pulses == 1u);
}

static void test_open_from_closing(void)
{
    rig_t r;
    rig_init_moving(&r, GATE_STATE_CLOSING);
    rig_run(&r, GATE_STATE_CLOSING, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSING, REACTION_MS);
    CHECK(r.pulses == 1u);
    CHECK(r.pulse_at[0] == LEAD_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    // Stopped: the second pulse waits for the gap after the first one.
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, PULSE_SPACING_MS);
    CHECK(r.pulses == 2u);
    CHECK(r.pulse_at[1] == LEAD_MS + PULSE_SPACING_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_OPENING, TRAVEL_MS);
    CHECK(gate_controller_busy(&r.ctl));
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 2u);
}

static void test_open_when_not_closed_is_noop(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_NOT_CLOSED, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    rig_run(&r, GATE_STATE_NOT_CLOSED, LONG_MS);
    CHECK(r.pulses == 0u);
}

// Final review I-1: OPEN while the gate opens sends no pulse but stays active until the gate is open, so
// whatever happens on the way is still reported: a stop counts as open, closing gets the reserved stop.
static void test_open_while_opening_watches_the_travel(void)
{
    {
        rig_t r;
        rig_init_moving(&r, GATE_STATE_OPENING);
        rig_command(&r, GATE_CMD_OPEN);
        rig_run(&r, GATE_STATE_OPENING, TRAVEL_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, TICK_MS);
        check_done(&r, GATE_ERR_NONE);
        CHECK(r.pulses == 0u);
    }
    {
        rig_t r;
        rig_init_moving(&r, GATE_STATE_OPENING);
        rig_command(&r, GATE_CMD_OPEN);
        rig_run(&r, GATE_STATE_OPENING, TRAVEL_MS / 4u);
        rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
        rig_run(&r, GATE_STATE_CLOSING, CONFIRM_RUN_MS);
        CHECK(r.pulses == 1u);
        check_done(&r, GATE_ERR_WRONG_DIRECTION);
    }
}

static void test_close_from_not_closed(void)
{
    const gate_motion_t motions[] = {GATE_MOTION_NONE, GATE_MOTION_OPENING};
    for (size_t i = 0; i < sizeof motions / sizeof motions[0]; i++) {
        rig_t r;
        rig_init(&r);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, motions[i], LEAD_MS);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, motions[i], REACTION_MS);
        CHECK(r.pulses == 1u);
        CHECK(r.pulse_at[0] == LEAD_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
        rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
        check_done(&r, GATE_ERR_NONE);
        CHECK(r.pulses == 1u);
    }
}

static void test_close_after_stopped_closing(void)
{
    rig_t r;
    rig_init(&r);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, REACTION_MS);
    CHECK(r.pulses == 1u);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    // Opening is the expected answer here; it is stopped once it is confirmed.
    const uint32_t opening_at = rig_elapsed(&r);
    rig_run(&r, GATE_STATE_OPENING, CONFIRM_RUN_MS);
    CHECK(r.pulses == 2u);
    CHECK(r.pulse_at[1] == opening_at + CORROBORATED_AFTER_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, PULSE_SPACING_MS);
    CHECK(r.pulses == 3u);
    CHECK(r.pulse_at[2] == r.pulse_at[1] + PULSE_SPACING_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
    CHECK(gate_controller_busy(&r.ctl));
    rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 3u);
}

static void test_close_from_opening(void)
{
    rig_t r;
    rig_init_moving(&r, GATE_STATE_OPENING);
    rig_run(&r, GATE_STATE_OPENING, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    // The decoder keeps reporting OPENING until the stop is confirmed; that is not a reversal.
    rig_run(&r, GATE_STATE_OPENING, PULSE_SPACING_MS - SETTLE_MS);
    CHECK(r.pulses == 1u);
    CHECK(r.pulse_at[0] == LEAD_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, REACTION_MS);
    CHECK(r.pulses == 2u);
    CHECK(r.pulse_at[1] == LEAD_MS + PULSE_SPACING_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
    rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 2u);
}

static void test_close_when_closed_is_noop(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run(&r, GATE_STATE_CLOSED, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    rig_run(&r, GATE_STATE_CLOSED, LONG_MS);
    CHECK(r.pulses == 0u);
}

// Final review I-1: CLOSE while the gate closes sends no pulse but stays active until the gate is closed:
// an obstacle reversal on the way is OBSTACLE, a stop short of closed is UNEXPECTED_STOP.
static void test_close_while_closing_watches_the_travel(void)
{
    {
        rig_t r;
        rig_init_moving(&r, GATE_STATE_CLOSING);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
        check_done(&r, GATE_ERR_NONE);
    }
    {
        rig_t r;
        rig_init_moving(&r, GATE_STATE_CLOSING);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
        rig_run(&r, GATE_STATE_OPENING, TICK_MS);
        check_done(&r, GATE_ERR_OBSTACLE);
    }
    {
        rig_t r;
        rig_init_moving(&r, GATE_STATE_CLOSING);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
        rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
        check_done(&r, GATE_ERR_UNEXPECTED_STOP);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
        CHECK(r.pulses == 0u);
    }
}

static void test_unknown_waits_then_acts(void)
{
    const gate_cmd_t cmds[] = {GATE_CMD_OPEN, GATE_CMD_CLOSE};
    const gate_state_t recognized[] = {GATE_STATE_CLOSED, GATE_STATE_NOT_CLOSED};
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++) {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_UNKNOWN, LEAD_MS);
        rig_command(&r, cmds[i]);
        rig_run(&r, GATE_STATE_UNKNOWN, GATE_UNKNOWN_WAIT_MS - TICK_MS);
        CHECK(gate_controller_busy(&r.ctl));
        CHECK(r.pulses == 0u);
        rig_run(&r, recognized[i], TICK_MS);
        CHECK(r.pulses == 1u);
        CHECK(gate_controller_busy(&r.ctl));
    }
}

static void test_unknown_times_out(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_UNKNOWN, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run(&r, GATE_STATE_UNKNOWN, GATE_UNKNOWN_WAIT_MS);
    CHECK(gate_controller_busy(&r.ctl));
    rig_run(&r, GATE_STATE_UNKNOWN, TICK_MS);
    check_done(&r, GATE_ERR_UNKNOWN_STATE);
    rig_run(&r, GATE_STATE_NOT_CLOSED, LONG_MS);
    CHECK(r.pulses == 0u);
    // A new command clears the error.
    rig_command(&r, GATE_CMD_OPEN);
    CHECK(gate_controller_last_error(&r.ctl) == GATE_ERR_NONE);
    rig_run(&r, GATE_STATE_NOT_CLOSED, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 0u);
}

static void test_open_wrong_direction_stops_once(void)
{
    // From CLOSED: the opening pulse makes the gate close.
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_OPEN);
        rig_run(&r, GATE_STATE_CLOSED, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
        const uint32_t closing_at = rig_elapsed(&r);
        rig_run(&r, GATE_STATE_CLOSING, CONFIRM_RUN_MS);
        CHECK(r.pulses == 2u);
        CHECK(r.pulse_at[1] == closing_at + CORROBORATED_AFTER_MS);
        check_done(&r, GATE_ERR_WRONG_DIRECTION);
        rig_run(&r, GATE_STATE_CLOSING, REACTION_MS);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
        CHECK(r.pulses == 2u);
    }
    // From CLOSING: stop, open, the gate closes again: the reserved third pulse stops it.
    {
        rig_t r;
        rig_init_moving(&r, GATE_STATE_CLOSING);
        rig_run(&r, GATE_STATE_CLOSING, LEAD_MS);
        rig_command(&r, GATE_CMD_OPEN);
        rig_run(&r, GATE_STATE_CLOSING, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, PULSE_SPACING_MS);
        CHECK(r.pulses == 2u);
        rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
        const uint32_t closing_at = rig_elapsed(&r);
        rig_run(&r, GATE_STATE_CLOSING, CONFIRM_RUN_MS);
        CHECK(r.pulses == 3u);
        CHECK(r.pulse_at[2] == closing_at + CORROBORATED_AFTER_MS);
        check_done(&r, GATE_ERR_WRONG_DIRECTION);
        rig_run(&r, GATE_STATE_CLOSING, LONG_MS);
        rig_run(&r, GATE_STATE_CLOSED, LONG_MS);
        CHECK(r.pulses == 3u);
    }
    // The gate stops by itself before the gap allows the stop pulse: no pulse, still an error.
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_OPEN);
        rig_run(&r, GATE_STATE_CLOSED, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, REACTION_MS);
        rig_run(&r, GATE_STATE_CLOSING, REACTION_MS);
        CHECK(r.pulses == 1u);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
        check_done(&r, GATE_ERR_WRONG_DIRECTION);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
        CHECK(r.pulses == 1u);
    }
    // Closing seen during the opening travel is also stopped once.
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_OPEN);
        rig_run(&r, GATE_STATE_CLOSED, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
        rig_run(&r, GATE_STATE_OPENING, TRAVEL_MS);
        rig_run(&r, GATE_STATE_CLOSING, CONFIRM_RUN_MS);
        CHECK(r.pulses == 2u);
        check_done(&r, GATE_ERR_WRONG_DIRECTION);
        rig_run(&r, GATE_STATE_CLOSING, LONG_MS);
        CHECK(r.pulses == 2u);
    }
}

static void test_close_obstacle_cancels(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run(&r, GATE_STATE_NOT_CLOSED, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
    rig_run(&r, GATE_STATE_OPENING, TICK_MS);
    check_done(&r, GATE_ERR_OBSTACLE);
    rig_run(&r, GATE_STATE_OPENING, LONG_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, LONG_MS);
    CHECK(r.pulses == 1u);
}

static void test_close_obstacle_before_closing_recognized(void)
{
    // Opening straight after the closing pulse.
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run(&r, GATE_STATE_NOT_CLOSED, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
        rig_run(&r, GATE_STATE_OPENING, TICK_MS);
        check_done(&r, GATE_ERR_OBSTACLE);
        rig_run(&r, GATE_STATE_OPENING, LONG_MS);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, LONG_MS);
        CHECK(r.pulses == 1u);
    }
    // After the stop pulse of an opening gate the decoder left OPENING and then reports it again.
    {
        rig_t r;
        rig_init_moving(&r, GATE_STATE_OPENING);
        rig_run(&r, GATE_STATE_OPENING, LEAD_MS);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run(&r, GATE_STATE_OPENING, SETTLE_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, REACTION_MS);
        rig_run(&r, GATE_STATE_OPENING, TICK_MS);
        check_done(&r, GATE_ERR_OBSTACLE);
        rig_run(&r, GATE_STATE_OPENING, LONG_MS);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, LONG_MS);
        CHECK(r.pulses == 1u);
    }
    // Opening after the last pulse of the three-pulse path.
    {
        rig_t r;
        rig_init(&r);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LEAD_MS);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
        rig_run(&r, GATE_STATE_OPENING, CONFIRM_RUN_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, PULSE_SPACING_MS);
        CHECK(r.pulses == 3u);
        rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
        rig_run(&r, GATE_STATE_OPENING, TICK_MS);
        check_done(&r, GATE_ERR_OBSTACLE);
        rig_run(&r, GATE_STATE_OPENING, LONG_MS);
        CHECK(r.pulses == 3u);
    }
}

static void test_short_reversal_then_stop(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run(&r, GATE_STATE_NOT_CLOSED, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
    rig_run(&r, GATE_STATE_OPENING, 1000u);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, LONG_MS);
    check_done(&r, GATE_ERR_OBSTACLE);
    CHECK(r.pulses == 1u);
}

static void test_no_confirm(void)
{
    // The gate does not react: the decoder keeps the old state.
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_OPEN);
        rig_run(&r, GATE_STATE_CLOSED, CONFIRM_LIMIT_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run(&r, GATE_STATE_CLOSED, TICK_MS);
        check_done(&r, GATE_ERR_NO_CONFIRM);
        rig_run(&r, GATE_STATE_CLOSED, LONG_MS);
        CHECK(r.pulses == 1u);
    }
    // The signal is never recognized after the pulse.
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run(&r, GATE_STATE_NOT_CLOSED, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, CONFIRM_LIMIT_MS - REACTION_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run(&r, GATE_STATE_UNKNOWN, TICK_MS);
        check_done(&r, GATE_ERR_NO_CONFIRM);
        rig_run(&r, GATE_STATE_NOT_CLOSED, LONG_MS);
        CHECK(r.pulses == 1u);
    }
}

// Ruling H (a): halfway is open, later wall-button moves are not ours.
static void test_wall_button_after_open_done(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSED, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_OPENING, TRAVEL_MS / 4u);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, LONG_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
    rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
    CHECK(r.pulses == 1u);
    check_done(&r, GATE_ERR_NONE);
}

// Ruling H (b): a stop before CLOSED that we did not cause ends CLOSE with an error.
static void test_wall_button_during_close(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run(&r, GATE_STATE_NOT_CLOSED, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    check_done(&r, GATE_ERR_UNEXPECTED_STOP);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
    CHECK(r.pulses == 1u);
}

// Ruling I: opening after our stop pulse during OPEN needs no more pulses.
static void test_open_reversed_by_stop_pulse_waits_for_stop(void)
{
    rig_t r;
    rig_init_moving(&r, GATE_STATE_CLOSING);
    rig_run(&r, GATE_STATE_CLOSING, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSING, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_OPENING, TRAVEL_MS);
    CHECK(gate_controller_busy(&r.ctl));
    CHECK(r.pulses == 1u);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 1u);
}

// Ruling I: last_motion was stale, the first pulse already closes; no more pulses.
static void test_close_with_stale_last_motion_closes_directly(void)
{
    rig_t r;
    rig_init(&r);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
    CHECK(gate_controller_busy(&r.ctl));
    CHECK(r.pulses == 1u);
    rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 1u);
}

static void test_travel_timeout(void)
{
    const gate_cmd_t cmds[] = {GATE_CMD_OPEN, GATE_CMD_CLOSE};
    const gate_state_t from[] = {GATE_STATE_CLOSED, GATE_STATE_NOT_CLOSED};
    const gate_state_t motion[] = {GATE_STATE_OPENING, GATE_STATE_CLOSING};
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++) {
        rig_t r;
        rig_init(&r);
        rig_run(&r, from[i], LEAD_MS);
        rig_command(&r, cmds[i]);
        rig_run(&r, from[i], REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
        rig_run(&r, motion[i], GATE_MAX_TRAVEL_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run(&r, motion[i], TICK_MS);
        check_done(&r, GATE_ERR_TRAVEL_TIMEOUT);
        rig_run(&r, motion[i], LONG_MS);
        CHECK(r.pulses == 1u);
    }
}

static void test_unknown_during_travel(void)
{
    // A short loss of recognition is normal.
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run(&r, GATE_STATE_NOT_CLOSED, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
        rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
        rig_run(&r, GATE_STATE_UNKNOWN, GATE_UNKNOWN_WAIT_MS - TICK_MS);
        rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
        rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
        check_done(&r, GATE_ERR_NONE);
        CHECK(r.pulses == 1u);
    }
    // A long loss of recognition ends the command without pulses.
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_CLOSE);
        rig_run(&r, GATE_STATE_NOT_CLOSED, REACTION_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
        rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
        rig_run(&r, GATE_STATE_UNKNOWN, GATE_UNKNOWN_WAIT_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run(&r, GATE_STATE_UNKNOWN, TICK_MS);
        check_done(&r, GATE_ERR_UNKNOWN_STATE);
        rig_run(&r, GATE_STATE_CLOSING, LONG_MS);
        rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
        CHECK(r.pulses == 1u);
    }
}

static void test_new_command_replaces_active(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSED, 200u);
    CHECK(r.pulses == 1u);
    rig_command(&r, GATE_CMD_CLOSE);
    CHECK(gate_controller_busy(&r.ctl));
    CHECK(gate_controller_last_error(&r.ctl) == GATE_ERR_NONE);
    // CLOSED is still the reading from before our pulse: it must not finish CLOSE.
    rig_run(&r, GATE_STATE_CLOSED, REACTION_MS);
    CHECK(gate_controller_busy(&r.ctl));
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    const uint32_t opening_at = rig_elapsed(&r);
    rig_run(&r, GATE_STATE_OPENING, CONFIRM_RUN_MS);
    CHECK(r.pulses == 2u);
    CHECK(r.pulse_at[1] == opening_at + CORROBORATED_AFTER_MS);
    rig_run(&r, GATE_STATE_OPENING, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, PULSE_SPACING_MS);
    CHECK(r.pulses == 3u);
    CHECK(r.pulse_at[2] == r.pulse_at[1] + PULSE_SPACING_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
    rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 3u);
}

// A due pulse never fires into UNKNOWN; it fires once the planned state is back.
static void test_pending_pulse_holds_in_unknown(void)
{
    rig_t r;
    rig_init_moving(&r, GATE_STATE_CLOSING);
    rig_run(&r, GATE_STATE_CLOSING, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSING, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LEAD_MS);  // second pulse due, gap not over
    CHECK(r.pulses == 1u);
    rig_run(&r, GATE_STATE_UNKNOWN, PULSE_SPACING_MS);
    CHECK(r.pulses == 1u);
    CHECK(gate_controller_busy(&r.ctl));
    const uint32_t back_at = rig_elapsed(&r);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    CHECK(r.pulses == 2u);
    CHECK(r.pulse_at[1] == back_at);
}

// The budget belongs to each command: a CLOSE after a failed CLOSE gets all three pulses.
static void test_budget_is_per_command(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_NOT_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run(&r, GATE_STATE_NOT_CLOSED, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS / 2u);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
    check_done(&r, GATE_ERR_UNEXPECTED_STOP);
    CHECK(r.pulses == 1u);

    rig_command(&r, GATE_CMD_CLOSE);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_OPENING, CONFIRM_RUN_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, PULSE_SPACING_MS);
    CHECK(r.pulses == 4u);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
    rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
}

// Random states, commands and relay refusals; rig_run checks the gap, the budget, no pulse in UNKNOWN
// and no pulse without an active command on every accepted pulse.
static void test_pulse_gap_and_budget(void)
{
    rig_t r;
    rig_init(&r);
    uint32_t seed = FUZZ_SEED;
    while (rig_elapsed(&r) < FUZZ_RUN_MS) {
        if (next_random(&seed) % FUZZ_COMMAND_ODDS == 0u) {
            rig_command(&r, (next_random(&seed) & 1u) ? GATE_CMD_OPEN : GATE_CMD_CLOSE);
        }
        r.relay_refuses = next_random(&seed) % FUZZ_COMMAND_ODDS == 0u;
        run_random_segment(&r, &seed);
    }
    CHECK(r.pulses > 0u);    // the scenario actually exercised pulses
    CHECK(r.refusals > 0u);  // and refusals
}

static void test_no_pulse_without_command(void)
{
    rig_t r;
    rig_init(&r);
    check_done(&r, GATE_ERR_NONE);
    uint32_t seed = FUZZ_SEED;
    while (rig_elapsed(&r) < NO_COMMAND_RUN_MS) {
        run_random_segment(&r, &seed);
    }
    CHECK(r.pulses == 0u);
    check_done(&r, GATE_ERR_NONE);
}

// A stop pulse on a gate whose motion was only just recognized waits until the motion is confirmed by
// a later edge: the decoder can hold one reading of a gate that has already stopped.
static void test_stop_pulse_waits_for_corroborated_motion(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_UNKNOWN, LEAD_MS);
    const uint32_t recognized_at = rig_elapsed(&r);
    rig_run(&r, GATE_STATE_CLOSING, TICK_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSING, CORROBORATED_AFTER_MS - 2u * TICK_MS);
    CHECK(r.pulses == 0u);
    CHECK(gate_controller_busy(&r.ctl));
    rig_run(&r, GATE_STATE_CLOSING, 2u * TICK_MS);
    CHECK(r.pulses == 1u);
    CHECK(r.pulse_at[0] == recognized_at + CORROBORATED_AFTER_MS);
}

// Review C2 at the controller: CLOSE with a stale "closing" memory sends its first pulse, which really
// closes the gate; an obstacle reversal then shows OPENING for less than the decoder hold time and the
// gate stops. The opening was never confirmed, so no stop pulse: the command fails.
static void test_unconfirmed_opening_after_reverse_pulse_gets_no_stop(void)
{
    rig_t r;
    rig_init(&r);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, REACTION_MS);
    CHECK(r.pulses == 1u);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_OPENING, GATE_STEADY_AFTER_MS - TICK_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
    check_done(&r, GATE_ERR_UNEXPECTED_STOP);
    CHECK(r.pulses == 1u);
}

// The same for OPEN: closing that is never confirmed gets no stop pulse, the command still fails.
static void test_unconfirmed_wrong_direction_gets_no_stop(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSED, REACTION_MS);
    CHECK(r.pulses == 1u);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    rig_run(&r, GATE_STATE_CLOSING, GATE_STEADY_AFTER_MS - TICK_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, LONG_MS);
    check_done(&r, GATE_ERR_WRONG_DIRECTION);
    CHECK(r.pulses == 1u);
}

// A refused pulse is not sent: it stays due, uses no budget, and goes out once the relay accepts it.
// The three-pulse CLOSE plan with every pulse refused first still sends exactly three pulses.
static void test_refused_pulses_stay_due_and_use_no_budget(void)
{
    rig_t r;
    rig_init(&r);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, LEAD_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    r.relay_refuses = true;
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, REACTION_MS);
    CHECK(r.pulses == 0u);
    CHECK(r.refusals == REACTION_MS / TICK_MS);
    r.relay_refuses = false;
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    CHECK(r.pulses == 1u);
    CHECK(r.pulse_at[0] == LEAD_MS + REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    r.relay_refuses = true;
    rig_run(&r, GATE_STATE_OPENING, CONFIRM_RUN_MS + REACTION_MS);
    CHECK(r.pulses == 1u);
    r.relay_refuses = false;
    rig_run(&r, GATE_STATE_OPENING, TICK_MS);
    CHECK(r.pulses == 2u);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    r.relay_refuses = true;
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, PULSE_SPACING_MS);
    CHECK(r.pulses == 2u);
    r.relay_refuses = false;
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, TICK_MS);
    CHECK(r.pulses == 3u);
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_CLOSING, TRAVEL_MS);
    rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 3u);
}

// The reserved stop for a wrong direction ends the command only once the relay has accepted it.
static void test_refused_wrong_direction_stop_keeps_the_command(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSED, REACTION_MS);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    r.relay_refuses = true;
    rig_run(&r, GATE_STATE_CLOSING, CONFIRM_RUN_MS + REACTION_MS);
    CHECK(r.pulses == 1u);
    CHECK(r.refusals > 0u);
    CHECK(gate_controller_busy(&r.ctl));
    CHECK(gate_controller_last_error(&r.ctl) == GATE_ERR_NONE);
    r.relay_refuses = false;
    rig_run(&r, GATE_STATE_CLOSING, TICK_MS);
    CHECK(r.pulses == 2u);
    check_done(&r, GATE_ERR_WRONG_DIRECTION);
}

// A due pulse waits at most GATE_CONFIRM_TIMEOUT_MS from its plan, whatever holds it back: a relay
// that keeps refusing, or a motion reading that keeps breaking into UNKNOWN and is never confirmed.
static void test_due_pulse_wait_is_bounded(void)
{
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_CLOSED, LEAD_MS);
        rig_command(&r, GATE_CMD_OPEN);
        r.relay_refuses = true;
        rig_run(&r, GATE_STATE_CLOSED, GATE_CONFIRM_TIMEOUT_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run(&r, GATE_STATE_CLOSED, TICK_MS);
        check_done(&r, GATE_ERR_NO_CONFIRM);
        CHECK(r.pulses == 0u);
    }
    {
        rig_t r;
        rig_init(&r);
        rig_run(&r, GATE_STATE_UNKNOWN, LEAD_MS);
        rig_command(&r, GATE_CMD_OPEN);
        for (unsigned i = 0; i < 10u; i++) {
            rig_run(&r, GATE_STATE_CLOSING, GATE_STEADY_AFTER_MS - TICK_MS);
            rig_run(&r, GATE_STATE_UNKNOWN, 2u * TICK_MS);
        }
        check_done(&r, GATE_ERR_NO_CONFIRM);
        CHECK(r.pulses == 0u);
    }
}

// Ruling R: the reserved stop for a wrong direction has no GATE_CONFIRM_TIMEOUT_MS limit. It stays due
// while the gate closes against OPEN, whether the relay refuses or the reading breaks into UNKNOWN.
static void rig_open_turning_wrong(rig_t *r)
{
    rig_init(r);
    rig_run(r, GATE_STATE_CLOSED, LEAD_MS);
    rig_command(r, GATE_CMD_OPEN);
    rig_run(r, GATE_STATE_CLOSED, REACTION_MS);
    rig_run(r, GATE_STATE_UNKNOWN, SETTLE_MS);
}

static void test_wrong_direction_stop_has_no_due_limit(void)
{
    // (a) The relay refuses past the 3 s limit, then accepts: the stop goes out.
    {
        rig_t r;
        rig_open_turning_wrong(&r);
        r.relay_refuses = true;
        rig_run(&r, GATE_STATE_CLOSING, 2u * GATE_CONFIRM_TIMEOUT_MS);
        CHECK(gate_controller_busy(&r.ctl));
        CHECK(gate_controller_last_error(&r.ctl) == GATE_ERR_NONE);
        CHECK(r.pulses == 1u);
        r.relay_refuses = false;
        rig_run(&r, GATE_STATE_CLOSING, TICK_MS);
        CHECK(r.pulses == 2u);
        check_done(&r, GATE_ERR_WRONG_DIRECTION);
    }
    // A long UNKNOWN stretch does not drop it either; the stop goes out once closing is confirmed again.
    {
        rig_t r;
        rig_open_turning_wrong(&r);
        r.relay_refuses = true;
        rig_run(&r, GATE_STATE_CLOSING, CONFIRM_RUN_MS);
        rig_run(&r, GATE_STATE_UNKNOWN, 2u * GATE_UNKNOWN_WAIT_MS);
        CHECK(gate_controller_busy(&r.ctl));
        r.relay_refuses = false;
        rig_run(&r, GATE_STATE_CLOSING, CONFIRM_RUN_MS);
        CHECK(r.pulses == 2u);
        check_done(&r, GATE_ERR_WRONG_DIRECTION);
    }
    // (b) The gate stops closing (here it reaches the closed limit) while the stop is refused: no pulse.
    {
        rig_t r;
        rig_open_turning_wrong(&r);
        r.relay_refuses = true;
        rig_run(&r, GATE_STATE_CLOSING, 2u * GATE_CONFIRM_TIMEOUT_MS);
        r.relay_refuses = false;
        rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
        check_done(&r, GATE_ERR_WRONG_DIRECTION);
        rig_run_lm(&r, GATE_STATE_CLOSED, GATE_MOTION_CLOSING, LONG_MS);
        CHECK(r.pulses == 1u);
    }
    // (c) The relay never accepts: the command ends after GATE_MAX_TRAVEL_MS from the plan.
    {
        rig_t r;
        rig_open_turning_wrong(&r);
        r.relay_refuses = true;
        rig_run(&r, GATE_STATE_CLOSING, GATE_MAX_TRAVEL_MS);
        CHECK(gate_controller_busy(&r.ctl));
        rig_run(&r, GATE_STATE_CLOSING, TICK_MS);
        check_done(&r, GATE_ERR_TRAVEL_TIMEOUT);
        CHECK(r.pulses == 1u);
    }
}

// Without a pulse function every pulse is refused: the command ends with an error, no pulse, no crash.
static void test_no_pulse_function_refuses_every_pulse(void)
{
    gate_controller_t c;
    gate_controller_init(&c, g_t0, NULL, NULL);
    gate_controller_command(&c, GATE_CMD_OPEN, g_t0);
    unsigned pulses = 0;
    for (uint32_t t = 0; t <= GATE_CONFIRM_TIMEOUT_MS + TICK_MS; t += TICK_MS) {
        pulses += gate_controller_step(&c, GATE_STATE_CLOSED, GATE_MOTION_NONE, false, g_t0 + t) ? 1u : 0u;
    }
    CHECK(pulses == 0u);
    CHECK(!gate_controller_busy(&c));
    CHECK(gate_controller_last_error(&c) == GATE_ERR_NO_CONFIRM);
}

// Ruling S, the reviewer's residual (hardening-review.md Minor 1): CLOSE while opening, the stop pulse
// waits for the motion to be confirmed; the wall button stops the gate in a lit half meanwhile, so the
// decoder still reports OPENING but its last edge grows old. Once it is older than GATE_FRESH_EDGE_MS the
// stop is withheld, and the command ends with UNEXPECTED_STOP when the decoder shows the stop.
static void test_stop_pulse_needs_a_fresh_edge(void)
{
    rig_t r;
    rig_init(&r);
    rig_run(&r, GATE_STATE_UNKNOWN, LEAD_MS);
    rig_run(&r, GATE_STATE_OPENING, TICK_MS);
    rig_command(&r, GATE_CMD_CLOSE);
    rig_run(&r, GATE_STATE_OPENING, CORROBORATED_AFTER_MS - 2u * TICK_MS - SETTLE_MS);
    r.edges_stale = true;  // the gate stopped in a lit half; the reading is held
    rig_run(&r, GATE_STATE_OPENING, SETTLE_MS + LONG_MS / 10u);
    CHECK(r.pulses == 0u);
    CHECK(gate_controller_busy(&r.ctl));
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, TICK_MS);
    check_done(&r, GATE_ERR_UNEXPECTED_STOP);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, LONG_MS);
    CHECK(r.pulses == 0u);
}

// A gap in the edges only holds the stop back: a fresh edge of the same blink lets it go out.
static void test_stop_pulse_goes_out_once_an_edge_is_fresh_again(void)
{
    rig_t r;
    rig_init_moving(&r, GATE_STATE_CLOSING);
    r.edges_stale = true;
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSING, LONG_MS / 10u);
    CHECK(r.pulses == 0u);
    r.edges_stale = false;
    rig_run(&r, GATE_STATE_CLOSING, TICK_MS);
    CHECK(r.pulses == 1u);
}

// A repeat of the running command (a double tap in Home) leaves the running plan alone. Replaced, OPEN
// would lose that its own stop left the gate halfway and finish there, as halfway counts as open.
static void test_repeated_command_keeps_the_running_plan(void)
{
    rig_t r;
    rig_init_moving(&r, GATE_STATE_CLOSING);
    rig_command(&r, GATE_CMD_OPEN);
    rig_run(&r, GATE_STATE_CLOSING, REACTION_MS);
    CHECK(r.pulses == 1u);
    rig_run(&r, GATE_STATE_UNKNOWN, SETTLE_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, TICK_MS);
    gate_controller_command(&r.ctl, GATE_CMD_OPEN, r.now_ms);  // the repeat, while our stop holds the gate
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_CLOSING, PULSE_SPACING_MS);
    CHECK(r.pulses == 2u);
    CHECK(r.pulse_at[1] == PULSE_SPACING_MS);  // the first one went out at 0
    CHECK(gate_controller_busy(&r.ctl));
    rig_run(&r, GATE_STATE_UNKNOWN, RECOGNIZE_MS);
    rig_run(&r, GATE_STATE_OPENING, TRAVEL_MS);
    rig_run_lm(&r, GATE_STATE_NOT_CLOSED, GATE_MOTION_OPENING, TICK_MS);
    check_done(&r, GATE_ERR_NONE);
    CHECK(r.pulses == 2u);
}

int main(void)
{
    const uint32_t starts[] = {0u, UINT32_MAX - WRAP_MARGIN_MS};
    for (size_t i = 0; i < sizeof starts / sizeof starts[0]; i++) {
        g_t0 = starts[i];
        RUN_TEST(test_open_from_closed);
        RUN_TEST(test_open_from_closing);
        RUN_TEST(test_open_when_not_closed_is_noop);
        RUN_TEST(test_close_from_not_closed);
        RUN_TEST(test_close_after_stopped_closing);
        RUN_TEST(test_close_from_opening);
        RUN_TEST(test_close_when_closed_is_noop);
        RUN_TEST(test_close_while_closing_watches_the_travel);
        RUN_TEST(test_open_while_opening_watches_the_travel);
        RUN_TEST(test_unknown_waits_then_acts);
        RUN_TEST(test_unknown_times_out);
        RUN_TEST(test_open_wrong_direction_stops_once);
        RUN_TEST(test_close_obstacle_cancels);
        RUN_TEST(test_close_obstacle_before_closing_recognized);
        RUN_TEST(test_short_reversal_then_stop);
        RUN_TEST(test_no_confirm);
        RUN_TEST(test_wall_button_after_open_done);
        RUN_TEST(test_wall_button_during_close);
        RUN_TEST(test_open_reversed_by_stop_pulse_waits_for_stop);
        RUN_TEST(test_close_with_stale_last_motion_closes_directly);
        RUN_TEST(test_travel_timeout);
        RUN_TEST(test_unknown_during_travel);
        RUN_TEST(test_new_command_replaces_active);
        RUN_TEST(test_repeated_command_keeps_the_running_plan);
        RUN_TEST(test_pending_pulse_holds_in_unknown);
        RUN_TEST(test_budget_is_per_command);
        RUN_TEST(test_pulse_gap_and_budget);
        RUN_TEST(test_no_pulse_without_command);
        RUN_TEST(test_stop_pulse_waits_for_corroborated_motion);
        RUN_TEST(test_unconfirmed_opening_after_reverse_pulse_gets_no_stop);
        RUN_TEST(test_unconfirmed_wrong_direction_gets_no_stop);
        RUN_TEST(test_refused_pulses_stay_due_and_use_no_budget);
        RUN_TEST(test_refused_wrong_direction_stop_keeps_the_command);
        RUN_TEST(test_due_pulse_wait_is_bounded);
        RUN_TEST(test_wrong_direction_stop_has_no_due_limit);
        RUN_TEST(test_no_pulse_function_refuses_every_pulse);
        RUN_TEST(test_stop_pulse_needs_a_fresh_edge);
        RUN_TEST(test_stop_pulse_goes_out_once_an_edge_is_fresh_again);
    }
    return MINITEST_RESULT();
}
