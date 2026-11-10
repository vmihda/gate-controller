#include "sim_console.h"

#include <cstdio>

#include <esp_check.h>
#include <esp_console.h>
#include <esp_system.h>

#include "gate_task.h"

namespace {

const char *const TAG = "sim_console";

// A dropped command returns non-zero, which the console reports.
int command_open(int, char **)
{
    return gate_task_command(GATE_CMD_OPEN) ? 0 : 1;
}

int command_close(int, char **)
{
    return gate_task_command(GATE_CMD_CLOSE) ? 0 : 1;
}

int command_status(int, char **)
{
    const gate_status_t status = gate_task_status();
    sim_console_report(status.state, status.busy, status.error);
    return 0;
}

int command_hang(int, char **)
{
    std::printf("hang: relay on, then a watchdog reset is expected within 1.5 s\n");
    gate_task_hang_with_relay_on();
    return 0;
}

// A software reset for the Wokwi bench: the board has no reset pin there, and the spec checks a reset during motion.
int command_restart(int, char **)
{
    std::printf("restart: software reset now\n");
    esp_restart();
}

struct command_entry {
    const char *name;
    const char *help;
    esp_console_cmd_func_t handler;
};

constexpr command_entry commands[] = {
    {"open", "Open the gate", command_open},
    {"close", "Close the gate", command_close},
    {"status", "Print the gate state, busy flag and last command error", command_status},
    {"hang", "Debug: hang the gate task with the relay on; expect a watchdog reset", command_hang},
    {"restart", "Debug: software reset of the device", command_restart},
};

}  // namespace

void sim_console_report(gate_state_t state, bool busy, gate_error_t error)
{
    std::printf("gate: state=%s busy=%d error=%s\n", gate_state_name(state), busy ? 1 : 0, gate_error_name(error));
}

esp_err_t sim_console_start()
{
    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "gate>";
    const esp_console_dev_usb_serial_jtag_config_t device_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_usb_serial_jtag(&device_config, &repl_config, &repl), TAG, "REPL");

    for (const command_entry &entry : commands) {
        esp_console_cmd_t command{};
        command.command = entry.name;
        command.help = entry.help;
        command.func = entry.handler;
        ESP_RETURN_ON_ERROR(esp_console_cmd_register(&command), TAG, "register %s", entry.name);
    }
    ESP_RETURN_ON_ERROR(esp_console_register_help_command(), TAG, "help command");
    return esp_console_start_repl(repl);
}
