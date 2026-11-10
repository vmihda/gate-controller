#include "driver/gpio.h"
#include "esp_check.h"

#include "emu_board.h"

void flash_out_init(void)
{
    /* Latch the level first so the pin never drives high on the way to being an output. */
    ESP_ERROR_CHECK(gpio_set_level(EMU_PIN_FLASH, 0));
    ESP_ERROR_CHECK(gpio_set_direction(EMU_PIN_FLASH, GPIO_MODE_OUTPUT));
}

void flash_out_set(bool on)
{
    gpio_set_level(EMU_PIN_FLASH, on ? 1 : 0);
}
