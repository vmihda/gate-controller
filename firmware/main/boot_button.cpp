#include "boot_button.h"

#include <cstdint>

#include <driver/gpio.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "board_config.h"
#include "button_press.h"
#include "clock_ms.h"

namespace {

const char *const TAG = "boot_button";

constexpr uint32_t poll_ms = 20;
constexpr uint32_t task_stack_bytes = 4096;
constexpr UBaseType_t task_priority = 2;

boot_button_callback_t s_on_short_press = nullptr;
boot_button_callback_t s_on_long_press = nullptr;

void button_task(void *)
{
    button_press_t press{};
    button_press_init(&press);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        switch (button_press_sample(&press, gpio_get_level(board::boot_pin) == 0, clock_ms())) {
        case BUTTON_EVENT_SHORT:
            s_on_short_press();
            break;
        case BUTTON_EVENT_LONG:
            s_on_long_press();
            break;
        case BUTTON_EVENT_NONE:
            break;
        }
    }
}

}  // namespace

esp_err_t boot_button_start(boot_button_callback_t on_short_press, boot_button_callback_t on_long_press)
{
    ESP_RETURN_ON_FALSE(on_short_press != nullptr && on_long_press != nullptr, ESP_ERR_INVALID_ARG, TAG, "callbacks");
    s_on_short_press = on_short_press;
    s_on_long_press = on_long_press;

    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << board::boot_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "BOOT input");
    return xTaskCreate(button_task, "boot_button", task_stack_bytes, nullptr, task_priority, nullptr) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}
