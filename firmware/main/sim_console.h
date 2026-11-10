// USB console of the GATE_SIM build (Wokwi and bench checks): open, close, status, hang, restart.
#pragma once

#include <esp_err.h>

#include "gate_controller.h"

[[nodiscard]] esp_err_t sim_console_start();

// Prints "gate: state=<STATE> busy=<0|1> error=<ERROR>". Matches gate_state_listener, so the GATE_SIM
// build reports every change with it; the status command prints the same line.
void sim_console_report(gate_state_t state, bool busy, gate_error_t error);
