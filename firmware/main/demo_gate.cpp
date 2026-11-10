#include "demo_gate.h"

#include <cstdint>

#include <esp_log.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>

#include "clock_ms.h"
#include "flash_in.h"
#include "nice_sim.h"
#include "relay_out.h"

namespace {

const char *const TAG = "demo_gate";

constexpr uint32_t wall_press_ms = 200;

// The model is advanced by whoever calls in: the gate task every 10 ms, the button task on a press. Each
// call reads the clock inside the lock, so the model never sees time go backwards.
portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
nice_sim_t s_sim{};
bool s_contact_closed = false;  // the Step-by-step input, from the relay pulse or the wall button
uint32_t s_contact_since_ms = 0;
uint32_t s_contact_hold_ms = 0;
bool s_reported_lit = false;  // the FLASH level last handed to the gate task

// Under s_lock: releases the contact once its hold time is over, then moves the model to now.
uint32_t advance()
{
    const uint32_t now_ms = clock_ms();
    if (s_contact_closed && now_ms - s_contact_since_ms >= s_contact_hold_ms) {
        nice_sim_set_pp(&s_sim, false, now_ms);
        s_contact_closed = false;
    }
    nice_sim_tick(&s_sim, now_ms);
    return now_ms;
}

bool close_contact(uint32_t hold_ms)
{
    taskENTER_CRITICAL(&s_lock);
    const uint32_t now_ms = advance();
    const bool accepted = !s_contact_closed;
    if (accepted) {
        nice_sim_set_pp(&s_sim, true, now_ms);
        s_contact_closed = true;
        s_contact_since_ms = now_ms;
        s_contact_hold_ms = hold_ms;
    }
    taskEXIT_CRITICAL(&s_lock);
    return accepted;
}

// Under s_lock: the FLASH level now, taken as handed to the gate task.
bool report_level(uint32_t now_ms)
{
    s_reported_lit = nice_sim_flash(&s_sim, now_ms);
    return s_reported_lit;
}

}  // namespace

// --- flash_in.h on the model ---

esp_err_t flash_in_init()
{
    const nice_sim_config_t config = {.travel_ms = NICE_SIM_DEFAULT_TRAVEL_MS, .chop_hz = 0, .chop_duty_pct = 0};
    taskENTER_CRITICAL(&s_lock);
    const uint32_t now_ms = clock_ms();
    nice_sim_init(&s_sim, &config, now_ms);
    report_level(now_ms);
    taskEXIT_CRITICAL(&s_lock);
    ESP_LOGW(TAG, "demo build: the gate is simulated, no relay output and no FLASH input");
    return ESP_OK;
}

bool flash_in_is_lit()
{
    taskENTER_CRITICAL(&s_lock);
    const bool lit = report_level(advance());
    taskEXIT_CRITICAL(&s_lock);
    return lit;
}

// The model without chopping changes FLASH at most once per blink half-period (200 ms or more), and the gate
// task asks every 10 ms, so one edge per call is enough. Its time is the time of the call.
bool flash_in_take_edge(flash_edge_t *edge)
{
    taskENTER_CRITICAL(&s_lock);
    const uint32_t now_ms = advance();
    const bool was_lit = s_reported_lit;
    const bool lit = report_level(now_ms);
    taskEXIT_CRITICAL(&s_lock);
    if (lit == was_lit) {
        return false;
    }
    *edge = {.lit = lit, .at_ms = now_ms};
    return true;
}

bool flash_in_take_overflow()
{
    return false;
}

void flash_in_discard_edges()
{
    taskENTER_CRITICAL(&s_lock);
    report_level(advance());
    taskEXIT_CRITICAL(&s_lock);
}

// --- relay_out.h on the model ---

esp_err_t relay_out_init()
{
    return ESP_OK;  // no relay output in the demo build
}

// Same 1 s task watchdog subscription as relay_out (sdkconfig.defaults); there is no relay, so no RTC
// watchdog backstop is armed.
esp_err_t relay_out_watch_this_task()
{
    return esp_task_wdt_add(nullptr);
}

void relay_out_feed_watchdogs()
{
    esp_task_wdt_reset();
}

// Refusals like relay_out's: no zero-length pulse, none while one is active, none long enough to reach
// the model's "always open" mark. The start-to-start spacing check of relay_out is not repeated here; the
// controller already spaces its pulses.
bool relay_out_pulse(uint32_t ms)
{
    return ms > 0 && ms < NICE_SIM_ALWAYS_OPEN_MS && close_contact(ms);
}

void demo_gate_press_wall_button()
{
    if (!close_contact(wall_press_ms)) {
        ESP_LOGW(TAG, "wall button: the contact is already closed");
    }
}
