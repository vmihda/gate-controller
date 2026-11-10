#include "gate_task.h"

#include <atomic>
#include <cstdint>

#include <esp_check.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "clock_ms.h"
#include "flash_decoder.h"
#include "flash_in.h"
#include "gate_config.h"
#include "relay_out.h"

namespace {

const char *const TAG = "gate_task";

constexpr TickType_t loop_period = pdMS_TO_TICKS(10);
constexpr uint32_t task_stack_bytes = 4096;
// Above the console, LED and button tasks; the relay timing itself is in RMT hardware.
constexpr UBaseType_t task_priority = 10;
constexpr UBaseType_t command_queue_length = 4;

QueueHandle_t s_commands = nullptr;
gate_state_listener s_listener = nullptr;
portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
gate_status_t s_status = {GATE_STATE_UNKNOWN, false, GATE_ERR_NONE};  // written only by the gate task
#if CONFIG_GATE_SIM
std::atomic<bool> s_hang_requested{false};
#endif

// The decoder needs non-decreasing times. An edge taken after the previous tick can carry an
// earlier time; it is moved to that tick. Wrap-safe.
uint32_t not_before(uint32_t at_ms, uint32_t floor_ms)
{
    return static_cast<int32_t>(at_ms - floor_ms) < 0 ? floor_ms : at_ms;
}

// The controller starts its pulses through this and learns whether relay_out accepted them; relay_out
// logs every refusal.
bool start_relay_pulse(void *)
{
    return relay_out_pulse(GATE_PULSE_MS);
}

void publish(const gate_status_t &status)
{
    taskENTER_CRITICAL(&s_status_lock);
    s_status = status;
    taskEXIT_CRITICAL(&s_status_lock);
    s_listener(status.state, status.busy, status.error);
}

#if CONFIG_GATE_SIM
[[noreturn]] void hang_with_relay_on()
{
    ESP_LOGW(TAG, "hang: relay pulse %s, watchdogs no longer fed",
             relay_out_pulse(GATE_PULSE_MS) ? "started" : "refused");
    while (true) {
        __asm__ __volatile__("" ::: "memory");  // a busy loop the compiler must keep
    }
}
#endif

class gate_loop {
public:
    void start()
    {
        flash_in_discard_edges();
        last_ms_ = clock_ms();
        flash_decoder_init(&decoder_, last_ms_);
        flash_decoder_on_level(&decoder_, flash_in_is_lit(), last_ms_);  // seed, see flash_decoder.h
        gate_controller_init(&controller_, last_ms_, start_relay_pulse, nullptr);
        publish(current_status());
    }

    void step()
    {
        feed_edges();
        last_ms_ = not_before(clock_ms(), last_ms_);
        flash_decoder_tick(&decoder_, last_ms_);
        const bool command_taken = take_commands();
#if CONFIG_GATE_SIM
        if (s_hang_requested.load()) {
            hang_with_relay_on();
        }
#endif

        const gate_state_t state = flash_decoder_state(&decoder_);
        // A refused pulse stays due in the controller and is asked for again on a later step.
        gate_controller_step(&controller_, state, flash_decoder_last_motion(&decoder_),
                             flash_decoder_motion_fresh(&decoder_, last_ms_), last_ms_);

        const gate_status_t status = current_status();
        if (gate_status_publish_due(&s_status, &status, command_taken)) {
            publish(status);
        }
    }

private:
    gate_status_t current_status() const
    {
        return {flash_decoder_state(&decoder_), gate_controller_busy(&controller_),
                gate_controller_last_error(&controller_)};
    }

    void feed_edges()
    {
        flash_edge_t edge{};
        while (flash_in_take_edge(&edge)) {
            last_ms_ = not_before(edge.at_ms, last_ms_);
            flash_decoder_on_level(&decoder_, edge.lit, last_ms_);
        }
        if (flash_in_take_overflow()) {
            // Edges were lost; the level now is what the decoder must end on.
            ESP_LOGW(TAG, "FLASH edge queue overflow");
            flash_decoder_on_level(&decoder_, flash_in_is_lit(), last_ms_);
        }
    }

    // True when at least one command was taken.
    bool take_commands()
    {
        bool taken = false;
        gate_cmd_t cmd{};
        while (xQueueReceive(s_commands, &cmd, 0) == pdTRUE) {
            gate_controller_command(&controller_, cmd, last_ms_);
            taken = true;
        }
        return taken;
    }

    flash_decoder_t decoder_{};
    gate_controller_t controller_{};
    uint32_t last_ms_ = 0;  // latest time given to the decoder and the controller
};

void gate_task(void *)
{
    ESP_ERROR_CHECK(relay_out_watch_this_task());
    gate_loop loop;
    loop.start();
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        relay_out_feed_watchdogs();
        loop.step();
        xTaskDelayUntil(&wake, loop_period);
    }
}

}  // namespace

esp_err_t gate_task_start(gate_state_listener on_change)
{
    ESP_RETURN_ON_FALSE(on_change != nullptr, ESP_ERR_INVALID_ARG, TAG, "listener");
    s_listener = on_change;
    s_commands = xQueueCreate(command_queue_length, sizeof(gate_cmd_t));
    ESP_RETURN_ON_FALSE(s_commands != nullptr, ESP_ERR_NO_MEM, TAG, "command queue");
    return xTaskCreate(gate_task, "gate", task_stack_bytes, nullptr, task_priority, nullptr) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

bool gate_task_command(gate_cmd_t cmd)
{
    if (s_commands == nullptr || xQueueSend(s_commands, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "gate task not running or busy, command dropped");
        return false;
    }
    return true;
}

gate_status_t gate_task_status()
{
    taskENTER_CRITICAL(&s_status_lock);
    const gate_status_t status = s_status;
    taskEXIT_CRITICAL(&s_status_lock);
    return status;
}

#if CONFIG_GATE_SIM
void gate_task_hang_with_relay_on()
{
    s_hang_requested.store(true);
}
#endif
