/* End-to-end bench test: the Nice model, the FLASH decoder and the command controller in one 1 ms loop. */
/* The decoder is fed every raw FLASH edge, the decoder and the controller tick every 10 ms, and a pulse */
/* requested by the controller closes the Step-by-step contact for GATE_PULSE_MS. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "flash_decoder.h"
#include "gate_config.h"
#include "gate_controller.h"
#include "minitest.h"
#include "nice_sim.h"
#include "pulse_spacing.h"

#define MAX_PULSES_LOGGED 16u
#define SETTLE_MS 2000u
#define COMMAND_LIMIT_MS 90000u
#define START_MS 5000u
#define NEAR_WRAP_START_MS (UINT32_MAX - 20000u)

typedef struct {
    nice_sim_t nice;
    flash_decoder_t decoder;
    gate_controller_t controller;
    uint32_t now;
    uint32_t steps;  /* 1 ms steps done: the 10 ms cadence must not depend on the wrapping clock */
    bool flash_level;
    bool relay_closed;
    uint32_t relay_end_ms;
    uint32_t pulse_starts[MAX_PULSES_LOGGED];
    uint32_t pulse_count;
    uint32_t min_pulse_spacing_ms;  /* smallest distance between two pulse starts, UINT32_MAX if fewer than two */
    pulse_spacing_t relay_spacing;   /* the check relay_out makes with its own clock */
    uint32_t relay_refusals;
    uint32_t late_stamp_pulse;  /* 1-based number of an accepted pulse whose start relay_out stamps late, 0 = none */
    uint32_t late_stamp_ms;     /* how late: the task was preempted right after the start */
    bool relay_down;            /* relay_out refuses every pulse while set */
} bench_t;

static bool bench_relay(void *ctx);

static void bench_init(bench_t *b, uint16_t chop_hz, uint8_t chop_duty_pct, uint32_t start_ms)
{
    const nice_sim_config_t cfg = {.travel_ms = 15000u, .chop_hz = chop_hz, .chop_duty_pct = chop_duty_pct};
    *b = (bench_t){.now = start_ms, .min_pulse_spacing_ms = UINT32_MAX};
    nice_sim_init(&b->nice, &cfg, b->now);
    flash_decoder_init(&b->decoder, b->now);
    b->flash_level = nice_sim_flash(&b->nice, b->now);
    flash_decoder_on_level(&b->decoder, b->flash_level, b->now);  /* seed, see flash_decoder.h */
    gate_controller_init(&b->controller, b->now, bench_relay, b);
    pulse_spacing_init(&b->relay_spacing);
}

/* relay_out reads its clock after the controller's step. The worst case without preemption: the check
 * reads the step time, the start is stamped GATE_PULSE_SPACING_MARGIN_MS later. relay_out must still
 * accept every pulse the controller asks for. */
static bool start_pulse(bench_t *b)
{
    if (b->relay_down || !pulse_spacing_allows(&b->relay_spacing, b->now)) {
        b->relay_refusals++;
        return false;
    }
    const bool late = b->pulse_count + 1u == b->late_stamp_pulse;
    pulse_spacing_started(&b->relay_spacing, b->now + (late ? b->late_stamp_ms : GATE_PULSE_SPACING_MARGIN_MS));
    if (b->pulse_count > 0u) {
        const uint32_t spacing = b->now - b->pulse_starts[b->pulse_count - 1u];
        if (spacing < b->min_pulse_spacing_ms) {
            b->min_pulse_spacing_ms = spacing;
        }
    }
    if (b->pulse_count < MAX_PULSES_LOGGED) {
        b->pulse_starts[b->pulse_count] = b->now;
    }
    b->pulse_count++;
    b->relay_closed = true;
    b->relay_end_ms = b->now + GATE_PULSE_MS;
    nice_sim_set_pp(&b->nice, true, b->now);
    return true;
}

/* The pulse function the controller calls: relay_out on the model. */
static bool bench_relay(void *ctx)
{
    return start_pulse(ctx);
}

static bool accept_pulse(void *ctx)
{
    (void)ctx;
    return true;
}

static void bench_step_1ms(bench_t *b)
{
    b->now++;
    b->steps++;
    if (b->relay_closed && (int32_t)(b->now - b->relay_end_ms) >= 0) {
        b->relay_closed = false;
        nice_sim_set_pp(&b->nice, false, b->now);
    }
    nice_sim_tick(&b->nice, b->now);
    const bool level = nice_sim_flash(&b->nice, b->now);
    if (level != b->flash_level) {
        b->flash_level = level;
        flash_decoder_on_level(&b->decoder, level, b->now);
    }
    if (b->steps % 10u == 0u) {
        flash_decoder_tick(&b->decoder, b->now);
        (void)gate_controller_step(&b->controller, flash_decoder_state(&b->decoder),
                                   flash_decoder_last_motion(&b->decoder), flash_decoder_motion_fresh(&b->decoder, b->now),
                                   b->now);  /* pulses go through bench_relay */
    }
}

static void bench_run(bench_t *b, uint32_t duration_ms)
{
    for (uint32_t i = 0; i < duration_ms; i++) {
        bench_step_1ms(b);
    }
}

/* A press of the wall button: it closes the same contact as the relay. */
static void wall_press(bench_t *b, uint32_t hold_ms)
{
    nice_sim_set_pp(&b->nice, true, b->now);
    bench_run(b, hold_ms);
    nice_sim_set_pp(&b->nice, false, b->now);
}

typedef struct {
    bool press;
    uint32_t run_ms;
} setup_step_t;

typedef struct {
    const char *name;
    setup_step_t steps[5];
    uint32_t step_count;
    bool stopped_midway;  /* OPEN leaves such a gate alone: stopped midway counts as open (spec, commands) */
} start_pose_t;

/* Poses reached with the wall button only: the controller is idle while they are set up. */
static const start_pose_t POSES[] = {
    {"closed", {{false, 0}}, 0, false},
    {"opening", {{true, 5000}}, 1, false},
    {"fully open", {{true, 16000}}, 1, false},
    {"stopped after opening", {{true, 5000}, {true, 3000}}, 2, true},
    {"closing", {{true, 16000}, {true, 5000}}, 2, false},
    {"stopped after closing", {{true, 16000}, {true, 5000}, {true, 3000}}, 3, true},
};

static void set_up_pose(bench_t *b, const start_pose_t *pose)
{
    bench_run(b, SETTLE_MS);
    for (uint32_t i = 0; i < pose->step_count; i++) {
        wall_press(b, 200);
        bench_run(b, pose->steps[i].run_ms);
    }
}

/* Runs until the controller finishes and the gate stops moving. */
static void run_command(bench_t *b, gate_cmd_t cmd)
{
    gate_controller_command(&b->controller, cmd, b->now);
    uint32_t waited = 0;
    while (waited < COMMAND_LIMIT_MS) {
        bench_run(b, 10);
        waited += 10;
        const nice_sim_state_t state = nice_sim_state(&b->nice);
        if (!gate_controller_busy(&b->controller) && (state == NICE_SIM_CLOSED || state == NICE_SIM_STOPPED)) {
            break;
        }
    }
    bench_run(b, 2000);  /* let the decoder see the final steady level */
}

/* run_command, with the same command sent again every repeat_ms for the first REPEAT_FOR_MS: Home may resend
 * a command once Wi-Fi is back, while the first one still runs. */
#define REPEAT_FOR_MS 20000u
static void run_repeated_command(bench_t *b, gate_cmd_t cmd, uint32_t repeat_ms)
{
    gate_controller_command(&b->controller, cmd, b->now);
    uint32_t waited = 0;
    while (waited < COMMAND_LIMIT_MS) {
        bench_run(b, 10);
        waited += 10;
        if (waited < REPEAT_FOR_MS && waited % repeat_ms == 0u) {
            gate_controller_command(&b->controller, cmd, b->now);
        }
        const nice_sim_state_t state = nice_sim_state(&b->nice);
        if (waited >= REPEAT_FOR_MS && !gate_controller_busy(&b->controller) &&
            (state == NICE_SIM_CLOSED || state == NICE_SIM_STOPPED)) {
            break;
        }
    }
    bench_run(b, 2000);
}

static void check_common_invariants(const bench_t *b)
{
    CHECK(b->relay_refusals == 0u);
    CHECK(!nice_sim_always_open_seen(&b->nice));
    CHECK(b->min_pulse_spacing_ms == UINT32_MAX || b->min_pulse_spacing_ms >= GATE_PULSE_SPACING_MS);
    CHECK(b->pulse_count <= GATE_MAX_PULSES);
    CHECK(nice_sim_last_pp_closure_ms(&b->nice) == GATE_PULSE_MS || b->pulse_count == 0u);
}

static void check_final(const bench_t *b, gate_cmd_t cmd, bool left_alone, int32_t position_before)
{
    if (left_alone) {
        CHECK(nice_sim_position_permille(&b->nice) == position_before);
        CHECK(flash_decoder_state(&b->decoder) == GATE_STATE_NOT_CLOSED);
        CHECK(b->pulse_count == 0u);
    } else if (cmd == GATE_CMD_OPEN) {
        CHECK(nice_sim_position_permille(&b->nice) == 1000);
        CHECK(flash_decoder_state(&b->decoder) == GATE_STATE_NOT_CLOSED);
    } else {
        CHECK(nice_sim_position_permille(&b->nice) == 0);
        CHECK(flash_decoder_state(&b->decoder) == GATE_STATE_CLOSED);
    }
    CHECK(gate_controller_last_error(&b->controller) == GATE_ERR_NONE);
    CHECK(!gate_controller_busy(&b->controller));
    check_common_invariants(b);
}

static void run_every_pose(gate_cmd_t cmd, uint16_t chop_hz, uint8_t chop_duty_pct, uint32_t start_ms)
{
    for (uint32_t i = 0; i < sizeof POSES / sizeof POSES[0]; i++) {
        bench_t b;
        bench_init(&b, chop_hz, chop_duty_pct, start_ms);
        set_up_pose(&b, &POSES[i]);
        const int32_t position_before = nice_sim_position_permille(&b.nice);
        run_command(&b, cmd);

        const int failures_before = minitest_failures;
        check_final(&b, cmd, cmd == GATE_CMD_OPEN && POSES[i].stopped_midway, position_before);
        if (minitest_failures != failures_before) {  /* name the scenario that failed */
            printf("  pose: %s, cmd %s, chop %u Hz, pulses %u, error %d, position %d\n", POSES[i].name,
                   cmd == GATE_CMD_OPEN ? "open" : "close", (unsigned)chop_hz, (unsigned)b.pulse_count,
                   (int)gate_controller_last_error(&b.controller), (int)nice_sim_position_permille(&b.nice));
        }
    }
}

static void test_open_from_every_pose(void) { run_every_pose(GATE_CMD_OPEN, 0, 0, START_MS); }

static void test_close_from_every_pose(void) { run_every_pose(GATE_CMD_CLOSE, 0, 0, START_MS); }

/* Chopping inside the supported envelope: 100 Hz / 30 % and the corner 50 Hz / 50 %. */
static void test_chopped_flash_open_and_close(void)
{
    run_every_pose(GATE_CMD_OPEN, 100, 30, START_MS);
    run_every_pose(GATE_CMD_CLOSE, 100, 30, START_MS);
    run_every_pose(GATE_CMD_OPEN, 50, 50, START_MS);
    run_every_pose(GATE_CMD_CLOSE, 50, 50, START_MS);
}

/* Margin check outside the supported envelope (spec: chopping from 50 Hz): documents the headroom only. */
static void test_chopped_flash_margin_50hz_30pct(void) { run_every_pose(GATE_CMD_CLOSE, 50, 30, START_MS); }

/* The setups and commands run across the 32-bit wrap of the millisecond clock. */
static void test_clock_wrap_open_and_close(void)
{
    run_every_pose(GATE_CMD_OPEN, 100, 30, NEAR_WRAP_START_MS);
    run_every_pose(GATE_CMD_CLOSE, 100, 30, NEAR_WRAP_START_MS);
}

static void test_obstacle_during_close_is_not_retried(void)
{
    bench_t b;
    bench_init(&b, 0, 0, START_MS);
    bench_run(&b, SETTLE_MS);
    wall_press(&b, 200);
    bench_run(&b, 16000);  /* fully open */

    gate_controller_command(&b.controller, GATE_CMD_CLOSE, b.now);
    bench_run(&b, 4000);
    CHECK(nice_sim_state(&b.nice) == NICE_SIM_CLOSING);
    const uint32_t pulses_before = b.pulse_count;
    CHECK(pulses_before == 1u);

    nice_sim_obstacle(&b.nice, b.now);
    CHECK(nice_sim_state(&b.nice) == NICE_SIM_REVERSING);
    bench_run(&b, 20000);

    CHECK(b.pulse_count == pulses_before);  /* no pulse once the reversal started */
    CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
    CHECK(nice_sim_position_permille(&b.nice) > 0 && nice_sim_position_permille(&b.nice) < 1000);
    CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_NOT_CLOSED);
    const gate_error_t error = gate_controller_last_error(&b.controller);
    CHECK(error == GATE_ERR_OBSTACLE || error == GATE_ERR_UNEXPECTED_STOP);
    CHECK(!gate_controller_busy(&b.controller));
    check_common_invariants(&b);
}

/* A command sent again while it runs never adds a pulse, from any pose and at any moment of the run. */
static void test_repeated_command_adds_no_pulse(void)
{
    static const uint32_t REPEAT_MS[] = {250u, 700u, 1300u};
    static const gate_cmd_t CMDS[] = {GATE_CMD_OPEN, GATE_CMD_CLOSE};
    for (uint32_t c = 0; c < sizeof CMDS / sizeof CMDS[0]; c++) {
        for (uint32_t i = 0; i < sizeof POSES / sizeof POSES[0]; i++) {
            bench_t once;
            bench_init(&once, 0, 0, START_MS);
            set_up_pose(&once, &POSES[i]);
            run_command(&once, CMDS[c]);
            for (uint32_t r = 0; r < sizeof REPEAT_MS / sizeof REPEAT_MS[0]; r++) {
                bench_t b;
                bench_init(&b, 0, 0, NEAR_WRAP_START_MS);
                set_up_pose(&b, &POSES[i]);
                const int32_t position_before = nice_sim_position_permille(&b.nice);
                run_repeated_command(&b, CMDS[c], REPEAT_MS[r]);

                const int failures_before = minitest_failures;
                CHECK(b.pulse_count <= once.pulse_count);
                check_final(&b, CMDS[c], CMDS[c] == GATE_CMD_OPEN && POSES[i].stopped_midway, position_before);
                if (minitest_failures != failures_before) {
                    printf("  pose: %s, cmd %s, repeat every %u ms, pulses %u (once: %u), error %d\n", POSES[i].name,
                           CMDS[c] == GATE_CMD_OPEN ? "open" : "close", (unsigned)REPEAT_MS[r],
                           (unsigned)b.pulse_count, (unsigned)once.pulse_count,
                           (int)gate_controller_last_error(&b.controller));
                }
            }
        }
    }
}

static void test_wall_button_is_followed_by_the_decoder(void)
{
    bench_t b;
    bench_init(&b, 0, 0, START_MS);
    bench_run(&b, SETTLE_MS);
    wall_press(&b, 200);
    bench_run(&b, 2000);
    CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_OPENING);
    bench_run(&b, 16000);
    CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_NOT_CLOSED);
    CHECK(nice_sim_position_permille(&b.nice) == 1000);

    wall_press(&b, 200);
    bench_run(&b, 2000);
    CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_CLOSING);
    wall_press(&b, 200);  /* the user stops the gate halfway */
    bench_run(&b, 2000);
    CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_NOT_CLOSED);
    CHECK(b.pulse_count == 0u);
    CHECK(!nice_sim_always_open_seen(&b.nice));
}

static void test_wall_button_stops_a_close_in_progress(void)
{
    bench_t b;
    bench_init(&b, 0, 0, START_MS);
    bench_run(&b, SETTLE_MS);
    wall_press(&b, 200);
    bench_run(&b, 16000);

    gate_controller_command(&b.controller, GATE_CMD_CLOSE, b.now);
    bench_run(&b, 5000);
    CHECK(nice_sim_state(&b.nice) == NICE_SIM_CLOSING);
    const uint32_t pulses_before = b.pulse_count;
    wall_press(&b, 200);
    bench_run(&b, 10000);

    CHECK(b.pulse_count == pulses_before);
    CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
    CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_UNEXPECTED_STOP);
    CHECK(!nice_sim_always_open_seen(&b.nice));
}

/* Review C1 end to end. FLASH is scripted: a 750 ms opening blink (inside the slow band) from 2000 ms,
 * rising at 9500, falling at 9875, and a wall stop that lights it at 9991. OPEN comes at 10000. The gate
 * is stopped after opening, so a pulse would close it: OPEN must not send one. */
static bool c1_flash(uint32_t t)
{
    const uint32_t blink_start_ms = 2000u;
    const uint32_t stop_ms = 9991u;
    const uint32_t period_ms = 750u;
    if (t < blink_start_ms) {
        return false;
    }
    return t >= stop_ms || (t - blink_start_ms) % period_ms < period_ms / 2u;
}

static void test_open_after_truncated_slow_period_sends_no_pulse(void)
{
    const uint32_t bases[] = {0u, UINT32_MAX - 9700u};  /* the second wraps between 9500 and 9875 */
    for (uint32_t i = 0; i < sizeof bases / sizeof bases[0]; i++) {
        flash_decoder_t decoder;
        gate_controller_t controller;
        const uint32_t base = bases[i];
        flash_decoder_init(&decoder, base);
        bool level = c1_flash(0u);
        flash_decoder_on_level(&decoder, level, base);
        gate_controller_init(&controller, base, accept_pulse, NULL);
        uint32_t pulses = 0;
        for (uint32_t t = 1; t <= 20000u; t++) {
            if (c1_flash(t) != level) {
                level = !level;
                flash_decoder_on_level(&decoder, level, base + t);
            }
            if (t % 10u == 0u) {
                flash_decoder_tick(&decoder, base + t);
                if (t == 10000u) {
                    gate_controller_command(&controller, GATE_CMD_OPEN, base + t);
                }
                if (gate_controller_step(&controller, flash_decoder_state(&decoder), flash_decoder_last_motion(&decoder),
                                         flash_decoder_motion_fresh(&decoder, base + t), base + t)) {
                    pulses++;
                }
            }
            CHECK(flash_decoder_state(&decoder) != GATE_STATE_CLOSING);
        }
        CHECK(pulses == 0u);
        CHECK(!gate_controller_busy(&controller));
        CHECK(gate_controller_last_error(&controller) == GATE_ERR_NONE);
        CHECK(flash_decoder_state(&decoder) == GATE_STATE_NOT_CLOSED);
    }
}

/* Review C2, the exact sequence on the unchanged model. Closing is recognized and stopped; then the
 * wall button opens for 100 ms and stops again while FLASH stays lit, so the decoder still remembers
 * closing. CLOSE plans "open, stop, close", but its first pulse closes the gate; an obstacle 230 ms
 * later reverses it for 1 s. That opening is the obstacle, not the answer to our pulse: no more pulses,
 * the gate stays where the reversal left it and the command fails. */
static void test_stale_last_motion_and_obstacle_send_no_more_pulses(void)
{
    const uint32_t starts[] = {0u, NEAR_WRAP_START_MS};
    for (uint32_t i = 0; i < sizeof starts / sizeof starts[0]; i++) {
        bench_t b;
        bench_init(&b, 0, 0, starts[i]);
        bench_run(&b, SETTLE_MS);
        wall_press(&b, 200);
        bench_run(&b, 16000);  /* fully open */
        wall_press(&b, 200);
        bench_run(&b, 5000);   /* closing, recognized */
        CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_CLOSING);
        wall_press(&b, 200);
        bench_run(&b, 2000);   /* stopped after closing */
        wall_press(&b, 100);
        bench_run(&b, 100);
        wall_press(&b, 100);   /* opened for 200 ms, stopped: FLASH never went dark */
        bench_run(&b, 2000);
        CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_NOT_CLOSED);
        CHECK(flash_decoder_last_motion(&b.decoder) == GATE_MOTION_CLOSING);  /* stale: the premise of C2 */

        gate_controller_command(&b.controller, GATE_CMD_CLOSE, b.now);
        bench_run(&b, 230);
        CHECK(b.pulse_count == 1u);
        CHECK(nice_sim_state(&b.nice) == NICE_SIM_CLOSING);  /* the plan expected opening */
        nice_sim_obstacle(&b.nice, b.now);
        bench_run(&b, 20000);

        const int failures_before = minitest_failures;
        CHECK(b.pulse_count == 1u);
        CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
        CHECK(nice_sim_position_permille(&b.nice) > 0);
        CHECK(!gate_controller_busy(&b.controller));
        CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_UNEXPECTED_STOP);
        check_common_invariants(&b);
        if (minitest_failures != failures_before) {
            printf("  start %u: pulses %u, error %d, position %d\n", (unsigned)starts[i], (unsigned)b.pulse_count,
                   (int)gate_controller_last_error(&b.controller), (int)nice_sim_position_permille(&b.nice));
        }
    }
}

/* Wokwi scenario 10 on the host, at every phase of the closing blink: an obstacle during a recognized
 * close. Nice reverses for 1 s and stops; the reversal is recognized, CLOSE ends with OBSTACLE, and no
 * pulse follows. When the reversal starts in a lit phase, that phase runs on into the first lit phase
 * of the opening blink. */
static void test_obstacle_at_any_closing_phase_is_reported(void)
{
    for (uint32_t phase = 0; phase < GATE_PERIOD_MAX_MS(FLASH_FAST_PERIOD_MS); phase += 5u) {
        bench_t b;
        bench_init(&b, 0, 0, START_MS);
        bench_run(&b, SETTLE_MS);
        wall_press(&b, 200);
        bench_run(&b, 16000);  /* fully open */
        gate_controller_command(&b.controller, GATE_CMD_CLOSE, b.now);
        bench_run(&b, 2500u + phase);
        CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_CLOSING);
        const uint32_t pulses_before = b.pulse_count;
        nice_sim_obstacle(&b.nice, b.now);
        bench_run(&b, 20000);

        const int failures_before = minitest_failures;
        CHECK(b.pulse_count == pulses_before);
        CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_OBSTACLE);
        CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
        check_common_invariants(&b);
        if (minitest_failures != failures_before) {
            printf("  obstacle %u ms into the close: error %d, pulses %u\n", (unsigned)(2500u + phase),
                   (int)gate_controller_last_error(&b.controller), (unsigned)b.pulse_count);
        }
    }
}

/* Codex hardening round 2, Critical: a relay refusal must not drop the reserved stop. Closing is
 * recognized; OPEN stops it. The wall button then opens the gate briefly and stops it (FLASH stays lit),
 * so OPEN's planned second pulse really closes the gate. relay_out is preempted for 100 ms right after
 * starting that pulse, so it stamps the start 100 ms late; the stop for the wrong direction comes
 * 1560 ms after the real start but 1460 ms after the stamp and is refused once. It must be retried:
 * the gate ends stopped, not closing against OPEN. */
static void test_refused_stop_pulse_is_retried(void)
{
    const uint32_t starts[] = {0u, UINT32_MAX - 23000u};
    for (uint32_t i = 0; i < sizeof starts / sizeof starts[0]; i++) {
        bench_t b;
        bench_init(&b, 0, 0, starts[i]);
        b.late_stamp_pulse = 2u;
        b.late_stamp_ms = 100u;
        bench_run(&b, SETTLE_MS);
        wall_press(&b, 200);
        bench_run(&b, 16000);  /* fully open */
        wall_press(&b, 200);
        bench_run(&b, 5000);   /* closing, recognized and confirmed */
        gate_controller_command(&b.controller, GATE_CMD_OPEN, b.now);
        bench_run(&b, 600);
        CHECK(b.pulse_count == 1u);
        wall_press(&b, 100);
        bench_run(&b, 100);
        wall_press(&b, 100);   /* opened 200 ms and stopped again: the next step closes */
        bench_run(&b, 6000);

        const int failures_before = minitest_failures;
        CHECK(b.relay_refusals >= 1u);
        CHECK(b.pulse_count == 3u);
        CHECK(b.min_pulse_spacing_ms >= GATE_PULSE_SPACING_MS);
        CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
        CHECK(!gate_controller_busy(&b.controller));
        CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_WRONG_DIRECTION);
        CHECK(!nice_sim_always_open_seen(&b.nice));
        if (minitest_failures != failures_before) {
            printf("  start %u: pulses %u, refusals %u, error %d, nice state %d\n", (unsigned)starts[i],
                   (unsigned)b.pulse_count, (unsigned)b.relay_refusals, (int)gate_controller_last_error(&b.controller),
                   (int)nice_sim_state(&b.nice));
        }
    }
}

/* Codex hardening round 2, Important: FLASH halves of 200/200, 200/200, 300/300 ms, repeating. The
 * decoder alternates CLOSING and UNKNOWN, so the motion is never confirmed. OPEN at 1000 ms plans a stop
 * pulse that can never fire; the command must still end, without a pulse. */
static bool flicker_flash(uint32_t t)
{
    const uint32_t k = t % 1400u;
    return k < 800u ? k % 400u < 200u : k - 800u < 300u;
}

static void test_flickering_motion_reading_ends_the_command(void)
{
    const uint32_t command_ms = 1000u;
    const uint32_t bases[] = {0u, UINT32_MAX - 3000u};
    for (uint32_t i = 0; i < sizeof bases / sizeof bases[0]; i++) {
        const uint32_t base = bases[i];
        flash_decoder_t decoder;
        gate_controller_t controller;
        flash_decoder_init(&decoder, base);
        bool level = false;
        flash_decoder_on_level(&decoder, level, base);
        gate_controller_init(&controller, base, accept_pulse, NULL);
        uint32_t pulses = 0;
        bool closing_seen = false;
        uint32_t done_after_ms = UINT32_MAX;
        for (uint32_t t = 0; t <= 120000u; t++) {
            if (flicker_flash(t) != level) {
                level = !level;
                flash_decoder_on_level(&decoder, level, base + t);
            }
            if (t % 10u != 0u) {
                continue;
            }
            flash_decoder_tick(&decoder, base + t);
            closing_seen = closing_seen || flash_decoder_state(&decoder) == GATE_STATE_CLOSING;
            if (t == command_ms) {
                gate_controller_command(&controller, GATE_CMD_OPEN, base + t);
            }
            if (gate_controller_step(&controller, flash_decoder_state(&decoder), flash_decoder_last_motion(&decoder),
                                     flash_decoder_motion_fresh(&decoder, base + t), base + t)) {
                pulses++;
            }
            if (t > command_ms && done_after_ms == UINT32_MAX && !gate_controller_busy(&controller)) {
                done_after_ms = t - command_ms;
            }
        }
        const int failures_before = minitest_failures;
        CHECK(closing_seen);  /* the premise: a reading that comes and goes */
        CHECK(pulses == 0u);
        CHECK(!gate_controller_busy(&controller));
        CHECK(done_after_ms <= GATE_CONFIRM_TIMEOUT_MS + 2u * 10u);
        CHECK(gate_controller_last_error(&controller) == GATE_ERR_NO_CONFIRM);
        if (minitest_failures != failures_before) {
            printf("  base %u: pulses %u, done after %u ms, error %d\n", (unsigned)base, (unsigned)pulses,
                   (unsigned)done_after_ms, (int)gate_controller_last_error(&controller));
        }
    }
}

/* Codex hardening round 3: the same OPEN that turns into closing as in
 * test_refused_stop_pulse_is_retried, but relay_out refuses every pulse until the reserved stop has been
 * due for GATE_CONFIRM_TIMEOUT_MS. The stop must stay due and go out once the relay recovers. */
static void set_up_open_that_closes(bench_t *b)
{
    bench_run(b, SETTLE_MS);
    wall_press(b, 200);
    bench_run(b, 16000);  /* fully open */
    wall_press(b, 200);
    bench_run(b, 5000);   /* closing, recognized and confirmed */
    gate_controller_command(&b->controller, GATE_CMD_OPEN, b->now);
    bench_run(b, 600);
    wall_press(b, 100);
    bench_run(b, 100);
    wall_press(b, 100);   /* opened 200 ms and stopped again: OPEN's next pulse closes */
}

/* The reserved stop is due: phase and kind are read from the controller for that. */
static bool wrong_stop_due(const bench_t *b)
{
    return b->controller.active && b->controller.phase == GATE_PHASE_PULSE &&
           b->controller.pulse_kind == GATE_KIND_WRONG_STOP;
}

static void test_stop_refused_past_the_due_limit_is_still_sent(void)
{
    const uint32_t starts[] = {0u, UINT32_MAX - 23000u};
    for (uint32_t i = 0; i < sizeof starts / sizeof starts[0]; i++) {
        bench_t b;
        bench_init(&b, 0, 0, starts[i]);
        set_up_open_that_closes(&b);
        bool due_seen = false;
        uint32_t planned_ms = 0;
        for (uint32_t t = 0; t < 8000u; t += 10u) {
            if (!due_seen && wrong_stop_due(&b)) {
                due_seen = true;
                planned_ms = b.controller.phase_ms;
            }
            b.relay_down = due_seen && b.now - planned_ms <= GATE_CONFIRM_TIMEOUT_MS;
            bench_run(&b, 10);
        }

        const int failures_before = minitest_failures;
        CHECK(due_seen);
        CHECK(b.relay_refusals > 0u);
        CHECK(b.pulse_count == 3u);
        CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
        CHECK(!gate_controller_busy(&b.controller));
        CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_WRONG_DIRECTION);
        if (minitest_failures != failures_before) {
            printf("  start %u: pulses %u, refusals %u, error %d, nice state %d\n", (unsigned)starts[i],
                   (unsigned)b.pulse_count, (unsigned)b.relay_refusals, (int)gate_controller_last_error(&b.controller),
                   (int)nice_sim_state(&b.nice));
        }
    }
}

/* The relay stays down until the wrongly closing gate reaches the closed limit: then there is nothing left
 * to stop. The command ends with WRONG_DIRECTION once FLASH shows the gate closed, not before, and no
 * pulse is sent. */
static void test_stop_refused_until_the_gate_closes_ends_without_pulse(void)
{
    const uint32_t starts[] = {0u, UINT32_MAX - 23000u};
    for (uint32_t i = 0; i < sizeof starts / sizeof starts[0]; i++) {
        bench_t b;
        bench_init(&b, 0, 0, starts[i]);
        set_up_open_that_closes(&b);
        uint32_t closed_at = 0;
        bool closed_seen = false;
        bool busy_when_closed = false;
        for (uint32_t t = 0; t < 20000u; t += 10u) {
            b.relay_down = wrong_stop_due(&b) || b.pulse_count >= 2u;
            bench_run(&b, 10);
            if (!closed_seen && nice_sim_state(&b.nice) == NICE_SIM_CLOSED) {
                closed_seen = true;
                closed_at = b.now;
                busy_when_closed = gate_controller_busy(&b.controller);
            }
        }

        const int failures_before = minitest_failures;
        CHECK(closed_seen);
        CHECK(busy_when_closed);  /* the stop was still due while the gate closed against OPEN */
        CHECK(b.pulse_count == 2u);
        CHECK(b.relay_refusals > 0u);
        CHECK(!gate_controller_busy(&b.controller));
        CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_WRONG_DIRECTION);
        CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_CLOSED);
        if (minitest_failures != failures_before) {
            printf("  start %u: pulses %u, closed at +%u, busy then %d, error %d\n", (unsigned)starts[i],
                   (unsigned)b.pulse_count, (unsigned)(closed_at - starts[i]), (int)busy_when_closed,
                   (int)gate_controller_last_error(&b.controller));
        }
    }
}

/* Codex hardening round 4: the reserved stop is refused for a long time. Meanwhile, at plan +4180 ms
 * while FLASH is lit, the wall button stops the closing, opens briefly and stops again (100 ms presses,
 * 100 ms apart). FLASH stays lit, so the decoder still reports the confirmed CLOSING for a while. The
 * relay recovers at plan +4690 ms: the stop must not go out, because no FLASH edge says the gate is
 * still moving; a pulse would start the stopped gate closing against OPEN. */
static void test_stop_after_the_gate_stopped_unseen_is_withheld(void)
{
    const uint32_t wall_at_ms = 4180u;
    const uint32_t relay_back_at_ms = 4690u;
    const uint32_t starts[] = {0u, UINT32_MAX - 23000u};
    for (uint32_t i = 0; i < sizeof starts / sizeof starts[0]; i++) {
        bench_t b;
        bench_init(&b, 0, 0, starts[i]);
        set_up_open_that_closes(&b);
        bool due_seen = false;
        bool walls_done = false;
        bool lit_at_wall = false;
        uint32_t planned_ms = 0;
        for (uint32_t t = 0; t < 12000u; t += 10u) {
            if (!due_seen && wrong_stop_due(&b)) {
                due_seen = true;
                planned_ms = b.controller.phase_ms;
            }
            const uint32_t since_plan = b.now - planned_ms;
            b.relay_down = due_seen && since_plan < relay_back_at_ms;
            if (due_seen && !walls_done && since_plan >= wall_at_ms) {
                walls_done = true;
                lit_at_wall = b.flash_level;
                wall_press(&b, 100);
                bench_run(&b, 100);
                wall_press(&b, 100);
                bench_run(&b, 100);
                wall_press(&b, 100);
            }
            bench_run(&b, 10);
        }

        const int failures_before = minitest_failures;
        CHECK(due_seen && walls_done && lit_at_wall);  /* the premises of the sequence */
        CHECK(b.pulse_count == 2u);
        CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
        CHECK(!gate_controller_busy(&b.controller));
        CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_WRONG_DIRECTION);
        if (minitest_failures != failures_before) {
            printf("  start %u: pulses %u, error %d, nice state %d, busy %d\n", (unsigned)starts[i],
                   (unsigned)b.pulse_count, (int)gate_controller_last_error(&b.controller),
                   (int)nice_sim_state(&b.nice), (int)gate_controller_busy(&b.controller));
        }
    }
}

/* Final review I-1, probe 1: CLOSE, then CLOSE again 4 s later while the gate closes (a double tap in
 * Home), then an obstacle. The second CLOSE must keep watching the travel, so the obstacle is reported. */
static void test_double_close_then_obstacle_is_reported(void)
{
    bench_t b;
    bench_init(&b, 0, 0, START_MS);
    bench_run(&b, SETTLE_MS);
    wall_press(&b, 200);
    bench_run(&b, 16000);  /* fully open */
    gate_controller_command(&b.controller, GATE_CMD_CLOSE, b.now);
    bench_run(&b, 4000);
    CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_CLOSING);
    gate_controller_command(&b.controller, GATE_CMD_CLOSE, b.now);
    bench_run(&b, 1000);
    CHECK(gate_controller_busy(&b.controller));
    nice_sim_obstacle(&b.nice, b.now);
    bench_run(&b, 20000);
    CHECK(b.pulse_count == 1u);
    CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_OBSTACLE);
    CHECK(!gate_controller_busy(&b.controller));
    CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
    check_common_invariants(&b);
}

/* Probe 2: the wall button starts closing, CLOSE comes from Home, then an obstacle. */
static void test_close_during_wall_closing_then_obstacle_is_reported(void)
{
    bench_t b;
    bench_init(&b, 0, 0, START_MS);
    bench_run(&b, SETTLE_MS);
    wall_press(&b, 200);
    bench_run(&b, 16000);  /* fully open */
    wall_press(&b, 200);
    bench_run(&b, 3000);   /* closing, recognized */
    CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_CLOSING);
    gate_controller_command(&b.controller, GATE_CMD_CLOSE, b.now);
    bench_run(&b, 1000);
    CHECK(gate_controller_busy(&b.controller));
    nice_sim_obstacle(&b.nice, b.now);
    bench_run(&b, 20000);
    CHECK(b.pulse_count == 0u);
    CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_OBSTACLE);
    CHECK(!gate_controller_busy(&b.controller));
    check_common_invariants(&b);
}

/* Probe 3, the OPEN mirror: the wall button starts opening, OPEN comes from Home, then the wall button
 * stops the gate and at once starts it closing. OPEN must keep watching and stop the closing once. */
static void test_open_during_wall_opening_then_closing_is_stopped(void)
{
    bench_t b;
    bench_init(&b, 0, 0, START_MS);
    bench_run(&b, SETTLE_MS);
    wall_press(&b, 200);
    bench_run(&b, 3000);   /* opening, recognized */
    CHECK(flash_decoder_state(&b.decoder) == GATE_STATE_OPENING);
    gate_controller_command(&b.controller, GATE_CMD_OPEN, b.now);
    bench_run(&b, 1000);
    CHECK(gate_controller_busy(&b.controller));
    wall_press(&b, 100);   /* stop */
    bench_run(&b, 200);
    wall_press(&b, 100);   /* close, before the decoder sees the stop */
    bench_run(&b, 10000);
    CHECK(b.pulse_count == 1u);
    CHECK(gate_controller_last_error(&b.controller) == GATE_ERR_WRONG_DIRECTION);
    CHECK(!gate_controller_busy(&b.controller));
    CHECK(nice_sim_state(&b.nice) == NICE_SIM_STOPPED);
    check_common_invariants(&b);
}

int main(void)
{
    RUN_TEST(test_open_from_every_pose);
    RUN_TEST(test_close_from_every_pose);
    RUN_TEST(test_chopped_flash_open_and_close);
    RUN_TEST(test_chopped_flash_margin_50hz_30pct);
    RUN_TEST(test_clock_wrap_open_and_close);
    RUN_TEST(test_obstacle_during_close_is_not_retried);
    RUN_TEST(test_repeated_command_adds_no_pulse);
    RUN_TEST(test_wall_button_is_followed_by_the_decoder);
    RUN_TEST(test_wall_button_stops_a_close_in_progress);
    RUN_TEST(test_open_after_truncated_slow_period_sends_no_pulse);
    RUN_TEST(test_stale_last_motion_and_obstacle_send_no_more_pulses);
    RUN_TEST(test_obstacle_at_any_closing_phase_is_reported);
    RUN_TEST(test_refused_stop_pulse_is_retried);
    RUN_TEST(test_flickering_motion_reading_ends_the_command);
    RUN_TEST(test_stop_refused_past_the_due_limit_is_still_sent);
    RUN_TEST(test_stop_refused_until_the_gate_closes_ends_without_pulse);
    RUN_TEST(test_stop_after_the_gate_stopped_unseen_is_withheld);
    RUN_TEST(test_double_close_then_obstacle_is_reported);
    RUN_TEST(test_close_during_wall_closing_then_obstacle_is_reported);
    RUN_TEST(test_open_during_wall_opening_then_closing_is_stopped);
    return MINITEST_RESULT();
}
