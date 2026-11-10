#include "flash_in.h"

#include <atomic>

#include <driver/gpio.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "board_config.h"
#include "clock_ms.h"

namespace {

const char *const TAG = "flash_in";

// Chopped FLASH (rectified 50 Hz) gives up to 200 edges/s and the queue is drained every 10 ms, so
// 64 entries cover a 300 ms stall of the gate task.
constexpr UBaseType_t queue_length = 64;

QueueHandle_t s_edges = nullptr;
std::atomic<bool> s_overflow{false};

// The optocoupler pulls D3 low while FLASH is lit.
bool lit_from_level(int level)
{
    return level == 0;
}

void on_edge(void *)
{
    const flash_edge_t edge = {.lit = lit_from_level(gpio_get_level(board::flash_pin)), .at_ms = clock_ms()};
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_edges, &edge, &woken) != pdTRUE) {
        s_overflow.store(true);
    }
    portYIELD_FROM_ISR(woken);
}

}  // namespace

esp_err_t flash_in_init()
{
    s_edges = xQueueCreate(queue_length, sizeof(flash_edge_t));
    ESP_RETURN_ON_FALSE(s_edges != nullptr, ESP_ERR_NO_MEM, TAG, "edge queue");

    // The pull-up is the external 22 kOhm resistor.
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << board::flash_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "D3 input");
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "GPIO ISR service");
    return gpio_isr_handler_add(board::flash_pin, on_edge, nullptr);
}

bool flash_in_is_lit()
{
    return lit_from_level(gpio_get_level(board::flash_pin));
}

bool flash_in_take_edge(flash_edge_t *edge)
{
    return xQueueReceive(s_edges, edge, 0) == pdTRUE;
}

bool flash_in_take_overflow()
{
    return s_overflow.exchange(false);
}

void flash_in_discard_edges()
{
    xQueueReset(s_edges);
    s_overflow.store(false);
}
