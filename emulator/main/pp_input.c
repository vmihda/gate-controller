#include <stdatomic.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"

#include "emu_board.h"

static atomic_uint_fast32_t s_pp_edges;

/* Counter only: no logging, no calls into flash code. */
static void IRAM_ATTR on_pp_edge(void *arg)
{
    (void)arg;
    atomic_fetch_add_explicit(&s_pp_edges, 1, memory_order_relaxed);
}

void pp_input_init(void)
{
    /* PP has its own external pull-up; the internal one is not needed. */
    const gpio_config_t pp = {
        .pin_bit_mask = 1ULL << EMU_PIN_PP,
        .mode = GPIO_MODE_INPUT,
    };
    const gpio_config_t obstacle = {
        .pin_bit_mask = 1ULL << EMU_PIN_OBSTACLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pp));
    ESP_ERROR_CHECK(gpio_config(&obstacle));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(EMU_PIN_PP, on_pp_edge, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type(EMU_PIN_PP, GPIO_INTR_ANYEDGE));  /* last, so setup noise is not counted */
}

bool pp_input_contact_closed(void)
{
    return gpio_get_level(EMU_PIN_PP) == 0;
}

bool pp_input_obstacle_pressed(void)
{
    return gpio_get_level(EMU_PIN_OBSTACLE) == 0;
}

uint32_t pp_input_edge_count(void)
{
    return (uint32_t)atomic_load_explicit(&s_pp_edges, memory_order_relaxed);
}
