// Pin assignment. Both boards are Seeed Studio XIAO with the same pad labels.
#pragma once

#include <driver/gpio.h>
#include <sdkconfig.h>

namespace board {

#if CONFIG_IDF_TARGET_ESP32S3
// Bench board, XIAO ESP32-S3.
inline constexpr gpio_num_t relay_pin = GPIO_NUM_2;  // D1, relay module IN (high level triggers), 10 kOhm to ground
inline constexpr gpio_num_t flash_pin = GPIO_NUM_4;  // D3, PC817 collector, 22 kOhm pull-up: low while FLASH is lit
inline constexpr gpio_num_t boot_pin = GPIO_NUM_0;
inline constexpr gpio_num_t led_pin = GPIO_NUM_21;
#define BOARD_HAS_RF_SWITCH 0
#elif CONFIG_IDF_TARGET_ESP32C6
// Gate board, XIAO ESP32-C6.
inline constexpr gpio_num_t relay_pin = GPIO_NUM_1;   // D1, relay module IN (high level triggers), 10 kOhm to ground
inline constexpr gpio_num_t flash_pin = GPIO_NUM_21;  // D3, PC817 collector, 22 kOhm pull-up: low while FLASH is lit
inline constexpr gpio_num_t boot_pin = GPIO_NUM_9;
inline constexpr gpio_num_t led_pin = GPIO_NUM_15;
// Low on both pins selects the built-in ceramic antenna.
#define BOARD_HAS_RF_SWITCH 1
inline constexpr gpio_num_t rf_switch_power_pin = GPIO_NUM_3;
inline constexpr gpio_num_t rf_switch_select_pin = GPIO_NUM_14;
#else
#error "Only the XIAO ESP32-S3 and XIAO ESP32-C6 boards are supported"
#endif

inline constexpr bool led_active_low = true;

}  // namespace board
