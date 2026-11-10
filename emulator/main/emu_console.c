/* Console commands on UART0: chop <hz> <duty>, travel <ms>, status.
 * chop and travel re-initialise the model, so they are accepted only while it is CLOSED. */
#include <inttypes.h>
#include <stdio.h>

#include "esp_check.h"
#include "esp_console.h"

#include "emu_board.h"
#include "emu_logic.h"

const char *emu_state_name(nice_sim_state_t state)
{
    switch (state) {
    case NICE_SIM_CLOSED:
        return "CLOSED";
    case NICE_SIM_OPENING:
        return "OPENING";
    case NICE_SIM_CLOSING:
        return "CLOSING";
    case NICE_SIM_STOPPED:
        return "STOPPED";
    case NICE_SIM_REVERSING:
        return "REVERSING";
    }
    return "?";
}

static int apply_config(const nice_sim_config_t *cfg)
{
    switch (emu_model_reconfigure(cfg)) {
    case EMU_RECONFIG_OK:
        break;
    case EMU_RECONFIG_NOT_CLOSED:
        printf("refused: the model must be CLOSED to change the configuration\n");
        return 1;
    case EMU_RECONFIG_CONTACT_HELD:
        printf("refused: the PP contact is closed; release it first (a re-init would lose the closure)\n");
        return 1;
    }
    if (cfg->chop_hz == 0) {
        printf("ok: travel %" PRIu32 " ms, FLASH steady\n", cfg->travel_ms);
    } else {
        uint32_t period_ms = 0;
        uint32_t on_ms = 0;
        emu_chop_effective(cfg->chop_hz, cfg->chop_duty_pct, &period_ms, &on_ms);
        printf("ok: travel %" PRIu32 " ms, chop %u Hz at %u %% = period %" PRIu32 " ms, on %" PRIu32 " ms\n",
               cfg->travel_ms, (unsigned)cfg->chop_hz, (unsigned)cfg->chop_duty_pct, period_ms, on_ms);
    }
    return 0;
}

static int cmd_chop(int argc, char **argv)
{
    if (argc < 2 || argc > 3) {
        printf("usage: chop <hz 0..%u> <duty 1..99>   (hz 0 = steady)\n", EMU_CHOP_MAX_HZ);
        return 1;
    }
    emu_status_t status;
    emu_model_status(&status);
    nice_sim_config_t cfg = status.cfg;
    if (!emu_parse_chop(argv[1], argc == 3 ? argv[2] : NULL, &cfg.chop_hz, &cfg.chop_duty_pct)) {
        printf("invalid arguments; usage: chop <hz 0..%u> <duty 1..99>   (hz 0 = steady)\n", EMU_CHOP_MAX_HZ);
        return 1;
    }
    return apply_config(&cfg);
}

static int cmd_travel(int argc, char **argv)
{
    emu_status_t status;
    emu_model_status(&status);
    nice_sim_config_t cfg = status.cfg;
    if (argc != 2 || !emu_parse_travel(argv[1], &cfg.travel_ms)) {
        printf("usage: travel <ms %u..%u>\n", EMU_TRAVEL_MIN_MS, EMU_TRAVEL_MAX_MS);
        return 1;
    }
    return apply_config(&cfg);
}

static int cmd_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    emu_status_t status;
    emu_model_status(&status);
    printf("state=%s position=%d/1000 travel_ms=%" PRIu32 " chop_hz=%u chop_duty=%u last_pp_closure_ms=%" PRIu32
           " always_open=%s pp_closures=%" PRIu32 " pp_edges=%" PRIu32 " pp_glitches=%" PRIu32
           " dropped_prints=%" PRIu32 "\n",
           emu_state_name(status.state), (int)status.position_permille, status.cfg.travel_ms,
           (unsigned)status.cfg.chop_hz, (unsigned)status.cfg.chop_duty_pct, status.last_closure_ms,
           status.always_open_seen ? "yes" : "no", status.closures, status.pp_edges, status.glitch_count, status.dropped_prints);
    return 0;
}

void emu_console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "emu>";
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    const esp_console_cmd_t commands[] = {
        {.command = "chop", .help = "Chop FLASH: chop <hz> <duty%>, hz 0 = steady (only while CLOSED)",
         .func = cmd_chop},
        {.command = "travel", .help = "Set full travel time: travel <ms> (only while CLOSED)", .func = cmd_travel},
        {.command = "status", .help = "Show model state, position, config, last closure", .func = cmd_status},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
