/* Wokwi custom chip: the Nice SNA2 control unit of the stage 1 bench.
 * The behaviour is emulator/components/nice_core/nice_sim.c, compiled in unchanged; this file only
 * connects the model to pins, a 1 ms timer and the A4988 stepper driver.
 *
 * Pins (see README.md):
 *   RELAY  in   device D1 (high = relay on = Step-by-step contact closed); pull-down like the 10 kOhm on D1
 *   PP     in   Step-by-step terminal, pull-up, low = closed; the "wall" button pulls it to GND
 *   OBST   in   "obstacle" button, pull-up, falling edge = obstacle
 *   FLASH  out  to device D3: low while FLASH is lit, as the PC817 collector with its pull-up
 *   STEP, DIR   to the A4988: one step per permille of travel, DIR high = opening
 *   CLOSED, OPEN        out  high at the closed / open limit
 *   ALWAYS_OPEN         out  sticky high once the "always open" rule (contact held 3 s) fired
 *   LONG_CLOSURE        out  sticky high once the contact stayed closed longer than 1.5 s
 *
 * Attributes: travel_ms (default 15000), chop_hz (0 = steady FLASH), chop_duty (percent high).
 *
 * Every contact closure is counted, including those the model ignores (a step during an obstacle reversal),
 * and the count is printed with each release. REPORT_DELAY_MS after an obstacle edge the chip prints
 * "nice: <n> closures so far"; an obstacle edge does nothing to a gate that is not closing, so a scenario
 * can press the obstacle button at the end and wait for that line to prove no closure came since. */
#include "wokwi-api.h"

#include <stdio.h>
#include <stdlib.h>

#include "nice_sim.h"

#define TICK_US 1000u
#define NANOS_PER_MS 1000000u
#define PERMILLE_FULL 1000
/* Five turns of a 200-step motor in full-step mode for the whole travel: one step per permille. */
#define STEPS_PER_TRAVEL 1000
/* Spec, readiness criteria: a pulse, even with hung firmware, must not close the contact longer than this. */
#define LONG_CLOSURE_MS 1500u
/* Later than the obstacle edge, so the scenario step after the button press can wait for the line. */
#define REPORT_DELAY_MS 10u

typedef struct {
    nice_sim_t sim;
    pin_t relay;
    pin_t pp;
    pin_t obstacle;
    pin_t flash;
    pin_t step;
    pin_t dir;
    pin_t closed_limit;
    pin_t open_limit;
    pin_t always_open;
    pin_t long_closure;
    int32_t motor_steps;  /* steps issued since start, 0 = closed */
    bool step_high;
    bool contact_closed;
    uint32_t contact_closed_ms;
    bool always_open_latched;
    bool long_closure_latched;
    nice_sim_state_t logged_state;
    uint32_t closures;  /* contact closures since start, relay and wall button */
    bool report_due;
    uint32_t report_at_ms;
} chip_t;

static uint32_t now_ms(void)
{
    return (uint32_t)(get_sim_nanos() / NANOS_PER_MS);
}

static const char *state_name(nice_sim_state_t state)
{
    switch (state) {
    case NICE_SIM_CLOSED:
        return "CLOSED";
    case NICE_SIM_OPENING:
        return "OPENING";
    case NICE_SIM_CLOSING:
        return "CLOSING";
    case NICE_SIM_STOPPED:
        return "STOPPED";
    case NICE_SIM_REVERSING:
        return "REVERSING";
    }
    return "?";
}

/* The relay contact and the wall button are in parallel on the Step-by-step terminal. */
static bool read_contact(const chip_t *chip)
{
    return pin_read(chip->relay) == HIGH || pin_read(chip->pp) == LOW;
}

/* Sticky alarm outputs: high from the first time the condition holds, logged once. */
static void latch(pin_t pin, bool *latched, bool condition, const char *message, uint32_t now)
{
    if (condition && !*latched) {
        *latched = true;
        pin_write(pin, HIGH);
        printf("nice: %s at %u ms\n", message, now);
    }
}

static void check_long_closure(chip_t *chip, uint32_t closed_for_ms, uint32_t now)
{
    latch(chip->long_closure, &chip->long_closure_latched, closed_for_ms > LONG_CLOSURE_MS,
          "LONG CLOSURE, contact closed longer than 1500 ms", now);
}

/* One step pulse per tick at most: STEP goes high on one tick and low on the next. */
static void drive_motor(chip_t *chip)
{
    if (chip->step_high) {
        pin_write(chip->step, LOW);
        chip->step_high = false;
        return;
    }
    const int32_t target = nice_sim_position_permille(&chip->sim) * STEPS_PER_TRAVEL / PERMILLE_FULL;
    if (target == chip->motor_steps) {
        return;
    }
    const bool opening = target > chip->motor_steps;
    pin_write(chip->dir, opening ? HIGH : LOW);
    pin_write(chip->step, HIGH);
    chip->step_high = true;
    chip->motor_steps += opening ? 1 : -1;
}

static void update_outputs(chip_t *chip, uint32_t now)
{
    const int32_t position = nice_sim_position_permille(&chip->sim);
    pin_write(chip->flash, nice_sim_flash(&chip->sim, now) ? LOW : HIGH);
    pin_write(chip->closed_limit, position == 0 ? HIGH : LOW);
    pin_write(chip->open_limit, position == PERMILLE_FULL ? HIGH : LOW);

    const nice_sim_state_t state = nice_sim_state(&chip->sim);
    if (state != chip->logged_state) {
        chip->logged_state = state;
        printf("nice: %s at %u ms, position %d/1000\n", state_name(state), now, (int)position);
    }
    latch(chip->always_open, &chip->always_open_latched, nice_sim_always_open_seen(&chip->sim),
          "ALWAYS OPEN fired", now);
}

static void on_contact_change(void *user_data, pin_t pin, uint32_t value)
{
    (void)pin;
    (void)value;
    chip_t *chip = user_data;
    const uint32_t now = now_ms();
    const bool closed = read_contact(chip);
    if (closed == chip->contact_closed) {
        return;  /* the other input of the parallel pair is still holding the contact */
    }
    chip->contact_closed = closed;
    nice_sim_set_pp(&chip->sim, closed, now);
    if (closed) {
        chip->contact_closed_ms = now;
        chip->closures++;
    } else {
        const uint32_t closed_for = nice_sim_last_pp_closure_ms(&chip->sim);
        printf("nice: contact released after %u ms, closure %u\n", closed_for, chip->closures);
        check_long_closure(chip, closed_for, now);
    }
    update_outputs(chip, now);
}

static void on_obstacle(void *user_data, pin_t pin, uint32_t value)
{
    (void)pin;
    (void)value;
    chip_t *chip = user_data;
    const uint32_t now = now_ms();
    printf("nice: obstacle at %u ms\n", now);
    nice_sim_obstacle(&chip->sim, now);
    update_outputs(chip, now);
    chip->report_due = true;
    chip->report_at_ms = now + REPORT_DELAY_MS;
}

static void on_tick(void *user_data)
{
    chip_t *chip = user_data;
    const uint32_t now = now_ms();
    nice_sim_tick(&chip->sim, now);
    if (chip->contact_closed) {
        check_long_closure(chip, now - chip->contact_closed_ms, now);
    }
    update_outputs(chip, now);
    drive_motor(chip);
    if (chip->report_due && (int32_t)(now - chip->report_at_ms) >= 0) {
        chip->report_due = false;
        printf("nice: %u closures so far\n", chip->closures);
    }
}

static void watch(pin_t pin, uint32_t edge, chip_t *chip,
                  void (*handler)(void *user_data, pin_t pin, uint32_t value))
{
    const pin_watch_config_t config = {.user_data = chip, .edge = edge, .pin_change = handler};
    pin_watch(pin, &config);
}

void chip_init(void)
{
    chip_t *chip = calloc(1, sizeof(chip_t));
    if (chip == NULL) {
        printf("nice: out of memory\n");
        return;
    }
    const nice_sim_config_t config = {
        .travel_ms = attr_read(attr_init("travel_ms", NICE_SIM_DEFAULT_TRAVEL_MS)),
        .chop_hz = (uint16_t)attr_read(attr_init("chop_hz", 0)),
        .chop_duty_pct = (uint8_t)attr_read(attr_init("chop_duty", 30)),
    };

    chip->relay = pin_init("RELAY", INPUT_PULLDOWN);
    chip->pp = pin_init("PP", INPUT_PULLUP);
    chip->obstacle = pin_init("OBST", INPUT_PULLUP);
    chip->flash = pin_init("FLASH", OUTPUT_HIGH);
    chip->step = pin_init("STEP", OUTPUT_LOW);
    chip->dir = pin_init("DIR", OUTPUT_LOW);
    chip->closed_limit = pin_init("CLOSED", OUTPUT_HIGH);
    chip->open_limit = pin_init("OPEN", OUTPUT_LOW);
    chip->always_open = pin_init("ALWAYS_OPEN", OUTPUT_LOW);
    chip->long_closure = pin_init("LONG_CLOSURE", OUTPUT_LOW);

    const uint32_t now = now_ms();
    nice_sim_init(&chip->sim, &config, now);
    chip->logged_state = nice_sim_state(&chip->sim);

    watch(chip->relay, BOTH, chip, on_contact_change);
    watch(chip->pp, BOTH, chip, on_contact_change);
    watch(chip->obstacle, FALLING, chip, on_obstacle);

    const timer_config_t timer_config = {.user_data = chip, .callback = on_tick};
    timer_start(timer_init(&timer_config), TICK_US, true);

    printf("nice: SNA2 model ready, travel %u ms, chop %u Hz at %u %%\n", (unsigned)config.travel_ms,
           (unsigned)config.chop_hz, (unsigned)config.chop_duty_pct);
}
