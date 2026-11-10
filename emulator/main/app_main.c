/* Bench emulator of the Nice SNA2 board: the same glue as virtual/wokwi/nice-sna2.chip.c on real hardware.
 * A 1 ms task samples PP and the obstacle button, steps the nice_sim model and drives FLASH and the stepper
 * target. The console task reads and reconfigures the model through emu_model_*() under a mutex; lines the model
 * prints go through a queue to a printer task. */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "emu_board.h"
#include "emu_logic.h"

#define MODEL_TASK_PRIORITY 10
#define MODEL_TASK_STACK 4096
#define PRINT_TASK_PRIORITY 3  /* above the console REPL (2), far below the model task */
#define PRINT_TASK_STACK 3072
#define PRINT_QUEUE_LEN 16
/* A glitch line is printed at most this often; the count in `status` stays exact. */
#define GLITCH_PRINT_MIN_INTERVAL_MS 200u

typedef enum { PRINT_CLOSURE, PRINT_GLITCH, PRINT_ALWAYS_OPEN } print_kind_t;

typedef struct {
    print_kind_t kind;
    uint32_t value_ms;
} print_event_t;

static nice_sim_t s_sim;
static nice_sim_config_t s_cfg = {.travel_ms = NICE_SIM_DEFAULT_TRAVEL_MS, .chop_hz = 0, .chop_duty_pct = 30};
static StaticSemaphore_t s_lock_storage;
static SemaphoreHandle_t s_lock;
static emu_glitch_t s_glitch;      /* guarded by s_lock */
static emu_history_t s_history;    /* guarded by s_lock; outlives every re-init of the model */
static bool s_pp_debounced_closed; /* guarded by s_lock */
static uint32_t s_dropped_prints;  /* guarded by s_lock */
static QueueHandle_t s_print_queue;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void emu_model_status(emu_status_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    out->state = nice_sim_state(&s_sim);
    out->position_permille = nice_sim_position_permille(&s_sim);
    out->cfg = s_cfg;
    out->last_closure_ms = s_history.last_closure_ms;
    out->always_open_seen = s_history.always_open_seen;
    out->closures = s_history.closures;
    out->pp_edges = pp_input_edge_count();
    out->glitch_count = emu_glitch_count(&s_glitch);
    out->dropped_prints = s_dropped_prints;
    xSemaphoreGive(s_lock);
}

emu_reconfig_t emu_model_reconfigure(const nice_sim_config_t *cfg)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const emu_reconfig_t result = emu_reconfigure_check(nice_sim_state(&s_sim) == NICE_SIM_CLOSED,
                                                        pp_input_contact_closed(), s_pp_debounced_closed);
    if (result == EMU_RECONFIG_OK) {
        s_cfg = *cfg;
        nice_sim_init(&s_sim, &s_cfg, now_ms());
    }
    xSemaphoreGive(s_lock);
    return result;
}

/* Model lines go through a queue to a printer task, so a console write that blocks (UART FIFO full while the REPL
 * prints) never stalls the 1 ms model task. Each line starts on its own line, never glued after the prompt. */
static void print_task(void *arg)
{
    (void)arg;
    for (;;) {
        print_event_t event;
        if (xQueueReceive(s_print_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (event.kind) {
        case PRINT_CLOSURE:
            printf("\npp_closure_ms=%" PRIu32 "\n", event.value_ms);
            break;
        case PRINT_GLITCH:
            printf("\npp_glitch_ms=%" PRIu32 "\n", event.value_ms);
            break;
        case PRINT_ALWAYS_OPEN:
            printf("\nALWAYS_OPEN fired: the contact was held for 3 s or longer\n");
            break;
        }
    }
}

/* Called with s_lock held. Never blocks; a full queue is counted. */
static void post_print(print_kind_t kind, uint32_t value_ms)
{
    const print_event_t event = {.kind = kind, .value_ms = value_ms};
    if (xQueueSend(s_print_queue, &event, 0) != pdTRUE) {
        s_dropped_prints++;
    }
}

static void model_task(void *arg)
{
    (void)arg;
    emu_debounce_t pp;
    emu_debounce_t obstacle;
    emu_debounce_init(&pp, false);
    emu_debounce_init(&obstacle, false);
    bool always_open_reported = false;
    uint32_t last_glitch_print_ms = 0;
    bool glitch_printed_once = false;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        if (xTaskDelayUntil(&wake, 1) != pdTRUE) {
            wake = xTaskGetTickCount();  /* woke late: resync instead of catching up with back-to-back samples */
        }
        const bool pp_raw = pp_input_contact_closed();
        const bool pp_changed = emu_debounce_update(&pp, pp_raw);
        const bool obstacle_changed = emu_debounce_update(&obstacle, pp_input_obstacle_pressed());

        xSemaphoreTake(s_lock, portMAX_DELAY);
        const uint32_t now = now_ms();
        if (pp_changed) {
            s_pp_debounced_closed = emu_debounce_stable(&pp);
            nice_sim_set_pp(&s_sim, s_pp_debounced_closed, now);
            if (!s_pp_debounced_closed) {
                emu_history_record_closure(&s_history, nice_sim_last_pp_closure_ms(&s_sim));
                post_print(PRINT_CLOSURE, s_history.last_closure_ms);
            }
        }
        const uint32_t glitch_ms = emu_glitch_update(&s_glitch, pp_raw, emu_debounce_stable(&pp));
        if (glitch_ms > 0 &&
            (!glitch_printed_once || now - last_glitch_print_ms >= GLITCH_PRINT_MIN_INTERVAL_MS)) {
            glitch_printed_once = true;
            last_glitch_print_ms = now;
            post_print(PRINT_GLITCH, glitch_ms);
        }
        if (obstacle_changed && emu_debounce_stable(&obstacle)) {
            nice_sim_obstacle(&s_sim, now);
        }
        nice_sim_tick(&s_sim, now);
        const bool flash_on = nice_sim_flash(&s_sim, now);
        const int32_t target = emu_steps_for_permille(nice_sim_position_permille(&s_sim));
        emu_history_note_always_open(&s_history, nice_sim_always_open_seen(&s_sim));
        if (s_history.always_open_seen && !always_open_reported) {
            post_print(PRINT_ALWAYS_OPEN, 0);
        }
        always_open_reported = s_history.always_open_seen;
        xSemaphoreGive(s_lock);

        flash_out_set(flash_on);
        stepper_set_target(target);
    }
}

void app_main(void)
{
    /* Outputs first: FLASH off and the driver disabled as early as possible. */
    flash_out_init();
    stepper_init();
    pp_input_init();

    s_lock = xSemaphoreCreateMutexStatic(&s_lock_storage);
    s_print_queue = xQueueCreate(PRINT_QUEUE_LEN, sizeof(print_event_t));
    emu_glitch_init(&s_glitch);
    emu_history_init(&s_history);
    nice_sim_init(&s_sim, &s_cfg, now_ms());

    const bool tasks_ok = s_print_queue != NULL &&
                          xTaskCreate(print_task, "print", PRINT_TASK_STACK, NULL, PRINT_TASK_PRIORITY, NULL) == pdPASS &&
                          xTaskCreate(model_task, "model", MODEL_TASK_STACK, NULL, MODEL_TASK_PRIORITY, NULL) == pdPASS;
    if (!tasks_ok) {
        printf("FATAL: cannot start the model tasks (out of memory)\n");
        abort();
    }
    emu_console_start();
    printf("nice emulator ready: travel %" PRIu32 " ms, type help\n", s_cfg.travel_ms);
}
