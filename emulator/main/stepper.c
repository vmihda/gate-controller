/* A4988 step generator: a 4 kHz hardware timer runs emu_stepper_tick() and applies its outputs to STEP, DIR and
 * ENABLE. 1/16 microstepping is set by MS1..MS3 tied high on the board. */
#include <stdatomic.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_check.h"

#include "emu_board.h"
#include "emu_logic.h"

#define TIMER_RESOLUTION_HZ 1000000u

static emu_stepper_t s_stepper;  /* touched only by the timer callback */
static atomic_int_fast32_t s_target;

static bool on_stepper_tick(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event, void *user_data)
{
    (void)timer;
    (void)event;
    (void)user_data;
    const emu_stepper_out_t out = emu_stepper_tick(&s_stepper, (int32_t)atomic_load(&s_target));
    gpio_set_level(EMU_PIN_DIR, out.dir_open ? 1 : 0);
    gpio_set_level(EMU_PIN_STEP, out.step ? 1 : 0);
    gpio_set_level(EMU_PIN_ENABLE, out.enable ? 0 : 1);
    return false;
}

static void output_init(int pin, int level)
{
    ESP_ERROR_CHECK(gpio_set_level(pin, level));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
}

void stepper_init(void)
{
    emu_stepper_init(&s_stepper);
    output_init(EMU_PIN_ENABLE, 1);  /* driver off at boot */
    output_init(EMU_PIN_STEP, 0);
    output_init(EMU_PIN_DIR, 0);

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    gptimer_handle_t timer = NULL;
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));
    const gptimer_alarm_config_t alarm = {
        .alarm_count = EMU_STEPPER_TICK_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    const gptimer_event_callbacks_t callbacks = {.on_alarm = on_stepper_tick};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

void stepper_set_target(int32_t steps)
{
    atomic_store(&s_target, steps);
}
