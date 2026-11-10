// The 10 ms gate loop: FLASH edges into the decoder, commands into the controller, relay pulses out,
// and every change of the published status to the listener.
#pragma once

#include <esp_err.h>
#include <sdkconfig.h>

#include "gate_controller.h"
#include "gate_status.h"

// Runs on the gate task, so it must return quickly.
using gate_state_listener = void (*)(gate_state_t state, bool busy, gate_error_t error);

// Needs relay_out_init() and flash_in_init() done. The listener is called once at start, then on every
// change and after every command taken (gate_status_publish_due).
[[nodiscard]] esp_err_t gate_task_start(gate_state_listener on_change);

// Thread-safe, from Matter or the console. A new command replaces the active one. False when the command was
// dropped (the gate task is not running or its queue is full); the drop is logged.
[[nodiscard]] bool gate_task_command(gate_cmd_t cmd);

// Thread-safe snapshot of what the listener was last given.
[[nodiscard]] gate_status_t gate_task_status();

#if CONFIG_GATE_SIM
// Bench check of the reset path: the gate task starts a relay pulse and then spins without feeding
// its watchdogs. The chip must reset within 1.5 s of the pulse start.
void gate_task_hang_with_relay_on();
#endif
