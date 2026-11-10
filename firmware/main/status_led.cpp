#include "status_led.h"

#include <atomic>
#include <cstdint>

#include <driver/gpio.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "board_config.h"
#include "clock_ms.h"
#include "gate_config.h"
#include "gate_task.h"
#include "status_policy.h"

namespace {

const char *const TAG = "status_led";

constexpr TickType_t refresh_period = pdMS_TO_TICKS(20);
constexpr uint32_t task_stack_bytes = 2560;
constexpr UBaseType_t task_priority = 2;

std::atomic<bool> s_resetting{false};
std::atomic<bool> s_commissioned{false};
std::atomic<bool> s_wifi_connected{false};
std::atomic<uint64_t> s_show_until_ms{0};  // uptime_ms(); 0 = never set, never showing

void write_led(bool on)
{
    gpio_set_level(board::led_pin, on == board::led_active_low ? 0 : 1);
}

void status_led_task(void *)
{
    bool started = false;
    while (true) {
        const gate_status_t gate = gate_task_status();
        started = status_policy_started(started, gate.state != GATE_STATE_UNKNOWN);
        const status_inputs_t inputs = {
            .resetting = s_resetting.load(),
            .command_error = gate.error != GATE_ERR_NONE,
            .commissioned = s_commissioned.load(),
            .wifi_connected = s_wifi_connected.load(),
            .started = started,
        };
        write_led(status_policy_led_lit(status_policy_select(&inputs), uptime_ms(), s_show_until_ms.load()));
        vTaskDelay(refresh_period);
    }
}

}  // namespace

esp_err_t status_led_start()
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << board::led_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "LED output");
    return xTaskCreate(status_led_task, "status_led", task_stack_bytes, nullptr, task_priority, nullptr) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

void status_led_show_for_a_moment()
{
    s_show_until_ms.store(uptime_ms() + GATE_LED_SHOW_MS);
}

void status_led_set_resetting()
{
    s_resetting.store(true);
}

void status_led_set_network(bool commissioned, bool wifi_connected)
{
    s_commissioned.store(commissioned);
    s_wifi_connected.store(wifi_connected);
}
