#include <initializer_list>

#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <sdkconfig.h>

#include "board_config.h"
#include "boot_button.h"
#include "flash_in.h"
#include "gate_task.h"
#include "relay_out.h"
#include "status_led.h"

#if CONFIG_GATE_SIM
#include "sim_console.h"
#else
#include "matter_app.h"
#endif
#if CONFIG_GATE_DEMO
#include "demo_gate.h"
#endif

namespace {

const char *const TAG = "app_main";

// Lets the very fast blink be seen before the device restarts.
constexpr TickType_t reset_signal_time = pdMS_TO_TICKS(1500);

// A full NVS, or one written by a newer format, cannot be opened: erase it and start again rather than
// boot-loop a unit far away. That loses the Home pairing, so it is an error in the log.
esp_err_t init_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(TAG, "NVS cannot be opened (%s): erasing it, the Home pairing is lost", esp_err_to_name(err));
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase");
        err = nvs_flash_init();
    }
    return err;
}

void select_builtin_antenna()
{
#if BOARD_HAS_RF_SWITCH
    for (const gpio_num_t pin : {board::rf_switch_power_pin, board::rf_switch_select_pin}) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);
    }
#endif
}

// After a fault reset the reason is logged, for the hang check (spec: reset by the task watchdog).
void log_reset_reason()
{
    switch (esp_reset_reason()) {
    case ESP_RST_TASK_WDT:
        ESP_LOGW(TAG, "restarted by the task watchdog");
        break;
    case ESP_RST_INT_WDT:
        ESP_LOGW(TAG, "restarted by the interrupt watchdog");
        break;
    case ESP_RST_WDT:
        ESP_LOGW(TAG, "restarted by the RTC or another hardware watchdog");
        break;
    case ESP_RST_PANIC:
        ESP_LOGW(TAG, "restarted after a panic");
        break;
    default:
        break;
    }
}

// Long BOOT press. GATE_SIM has no Matter state to erase and only restarts.
void factory_reset_with_signal()
{
    ESP_LOGW(TAG, "Factory reset");
    status_led_set_resetting();
    vTaskDelay(reset_signal_time);
#if CONFIG_GATE_SIM
    esp_restart();
#else
    matter_app_factory_reset();
#endif
}

// Short BOOT press: the wall button of the simulated gate in the demo build, else the LED shows the state.
#if CONFIG_GATE_DEMO
constexpr boot_button_callback_t short_press = demo_gate_press_wall_button;
#else
constexpr boot_button_callback_t short_press = status_led_show_for_a_moment;
#endif

}  // namespace

extern "C" void app_main()
{
    // First of all: D1 low and owned by relay_out (the demo build has no relay, see demo_gate.h).
    ESP_ERROR_CHECK(relay_out_init());
    log_reset_reason();
    ESP_ERROR_CHECK(init_nvs());  // after relay_out_init: D1 is already driven low
    select_builtin_antenna();

    ESP_ERROR_CHECK(flash_in_init());
#if CONFIG_GATE_SIM
    ESP_ERROR_CHECK(gate_task_start(sim_console_report));
    status_led_set_network(true, true);
#else
    // Matter first, so the gate task's first status already has the endpoint to go to.
    ESP_ERROR_CHECK(matter_app_start(gate_task_command));
    ESP_ERROR_CHECK(gate_task_start(matter_app_publish));
#endif
    ESP_ERROR_CHECK(status_led_start());
    ESP_ERROR_CHECK(boot_button_start(short_press, factory_reset_with_signal));
#if CONFIG_GATE_SIM
    ESP_ERROR_CHECK(sim_console_start());
#endif
}
