// Drives the board LED with the pattern status_policy picks from the device state.
#pragma once

#include <esp_err.h>

[[nodiscard]] esp_err_t status_led_start();
// Short BOOT press: a healthy device lights the LED steadily for 3 s.
void status_led_show_for_a_moment();
void status_led_set_resetting();
// Matter state. GATE_SIM has no Matter and reports both as true.
void status_led_set_network(bool commissioned, bool wifi_connected);
