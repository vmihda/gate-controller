// Matter node with one gate endpoint of the device type picked by GATE_ENDPOINT_KIND (Closure, Window
// Covering or Door Lock). Home shows only the gate: open and close, open or closed, and a command error.
#pragma once

#include <esp_err.h>

#include "gate_controller.h"

// Called on the Matter task for every open or close command from Home. False when the command was not
// taken (for gate_task_command: dropped); Home is then told the command failed.
using matter_command_handler = bool (*)(gate_cmd_t cmd);

// After nvs_flash_init(). Starts the Matter stack; the setup code is printed once the server is ready.
[[nodiscard]] esp_err_t matter_app_start(matter_command_handler on_command);

// Matches gate_state_listener: called on the gate task, returns at once. The newest status is applied to
// the gate endpoint on the Matter task; statuses that arrive before that are superseded.
void matter_app_publish(gate_state_t state, bool busy, gate_error_t error);

// Erases the Matter fabrics and Wi-Fi credentials, then restarts. Does not return.
[[noreturn]] void matter_app_factory_reset();
