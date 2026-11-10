// BOOT button: a short press and a long press, each with its own callback.
// Callbacks run on the button task, so they may block briefly.
#pragma once

#include <esp_err.h>

using boot_button_callback_t = void (*)();

[[nodiscard]] esp_err_t boot_button_start(boot_button_callback_t on_short_press, boot_button_callback_t on_long_press);
