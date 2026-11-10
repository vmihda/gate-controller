# Virtual bench in Wokwi

The simulation runs the real device firmware (the `GATE_SIM` build for the XIAO ESP32-S3). The Nice SNA2 board is a custom chip, `nice-sna2`, built from the same `emulator/components/nice_core/nice_sim.c` that the host tests check. `nice-sna2.chip.c` uses that file as is and only connects it to the pins, a 1 ms timer and the A4988 driver.

## Circuit

| From | To | Meaning |
|---|---|---|
| Device D1 (GPIO2) | `nice:RELAY` | high = relay on = Step-by-step contact closed |
| `nice:FLASH` | Device D3 (GPIO4) | low = FLASH lit |
| "Wall" button | `nice:PP` → GND | the wall button, closes the same Step-by-step input |
| "Obstacle" button | `nice:OBST` → GND | an obstacle, acts only while closing |
| `nice:STEP`, `nice:DIR` | A4988 → stepper | one step per mille of travel, full travel 1000 steps (5 turns), DIR high = opening |
| `nice:CLOSED`, `nice:OPEN` | LEDs | the gate is at the "closed" / "open" limit |
| `nice:ALWAYS_OPEN` | red LED | the "always open" rule fired (contact closed for 3 s). Latched |
| `nice:LONG_CLOSURE` | orange LED | the contact was closed for longer than 1.5 s. Latched |

There is no relay or optocoupler in the simulation. The chip's `RELAY` input has a pull-down, as D1 has a 10 kOhm resistor to ground on the board, so the contact stays open while the device resets. The relay contact and the wall button are in parallel: the contact is closed while D1 is high or the button is pressed. The chip's `FLASH` output drives the level that the PC817 collector and its pull-up would give on the board: low while FLASH is lit.

Chip attributes in `diagram.json`: `travel_ms` is the full travel time (default 15000), `chop_hz` is the chopping frequency of a lit FLASH (0 means a steady signal), `chop_duty` is the chopping duty in percent. Chip messages (`nice: OPENING at … ms`, `nice: contact released after … ms, closure N`) go to the `wokwi-cli` console with the `[chip-nice-sna2]` prefix. The chip counts every contact closure, from the relay and from the wall button, including those the model ignores (a step during the reversal after an obstacle). 10 ms after the "Obstacle" button is pressed, the chip prints `nice: N closures so far`. If the gate is not closing at that moment, the press changes nothing, so a scenario can press it at the end and wait for the line with the expected count.

The device console is USB Serial/JTAG (the board's `serialInterface`). Commands: `open`, `close`, `status`, `hang`, `restart`. Every change and every `status` command prints `gate: state=<STATE> busy=<0|1> error=<ERROR>`. `restart` exists only in the `GATE_SIM` build: the XIAO board in Wokwi has no reset pin, and the scenarios need a reset during motion.

## What Wokwi does not check

Voltages and currents, the relay module, the PC817 optocoupler and the FLASH input circuit (resistor, diode, RC filter), pull resistors on the board, contact bounce, power, Matter, Wi-Fi and Home. Instead of a power-on reset or the RESET button, the scenarios use the software reset `restart` and the watchdog reset after `hang`. The pulse length, and whether D1 glitches at boot, need a logic analyzer on the real board.

A software reset (`esp_restart`, a panic, the watchdogs) on the ESP32-S3 and ESP32-C6 does not reset the RMT peripheral: a pulse already started goes on through the reboot. So `relay_out_init()` holds D1 as a plain GPIO output at low level, resets the RMT peripheral, and hands D1 to RMT only 1 s after it starts. The rest of an old pulse ends at most 819 ms after its start (500 ms in practice), so by then it can no longer reach D1. The Wokwi RMT model does not reset the peripheral and continues the old pulse after boot, and esp_timer is not zeroed by a software reset. In Wokwi, therefore, `16-reset-during-pulse` tests only the 1 s delay. The peripheral reset can be checked only on the board, with a logic analyzer: `open`, then `restart` during the pulse; after the contact opens there must be no second high level on D1.

## Scenarios

| File | What it checks |
|---|---|
| `01-open-from-closed` | OPEN from "closed": one pulse, the gate opens to the limit |
| `02-close-from-closed` | CLOSE from "closed": no pulse |
| `03-open-from-open` | OPEN from "open": no pulse |
| `04-close-from-open` | CLOSE from "open": one pulse, the gate closes |
| `05-open-from-stopped` | OPEN with the gate stopped halfway: it counts as open, no pulse |
| `06-close-from-stopped-after-opening` | CLOSE, stopped halfway after opening: one pulse |
| `07-close-from-stopped-after-closing` | CLOSE, stopped halfway after closing: three pulses (open, stop, close) |
| `08-open-while-closing` | OPEN while closing: stop, then open |
| `09-close-while-opening` | CLOSE while opening: stop, then close |
| `10-obstacle-during-closing` | an obstacle during CLOSE: error `OBSTACLE`, no more pulses, also none during the reversal: the closure count at the end is 2 |
| `11-obstacle-right-after-pulse` | an obstacle right after the pulse, before closing is recognized: the reversal reaches the limit before opening is recognized, so the error is `UNEXPECTED_STOP`; no more pulses, the closure count at the end is 2 |
| `12-wall-during-open` | the wall button stops the gate during OPEN: the command ends without an error |
| `13-wall-during-close` | the wall button stops the gate during CLOSE: error `UNEXPECTED_STOP` |
| `14-reset-while-opening` | a reset while opening, then OPEN while the state is still unknown |
| `15-reset-while-closing` | a reset during CLOSE: no pulses after boot, the gate closes by itself |
| `16-reset-during-pulse` | a reset during a pulse: after boot D1 stays low, no second step, the gate opens to the limit |
| `17-hang-during-pulse` | `hang` during a pulse: the pulse is still 500 ms, a watchdog reset, "always open" does not fire |
| `18-hang-with-relay-on` | `hang` turns the relay on by itself and hangs: the same result |
| `chop-open-close` | a chopped FLASH: OPEN and CLOSE. It runs three times: chopping at 100 Hz / 30 %, at 50 Hz / 50 % (the corner of the supported envelope: 10 ms gaps) and at 50 Hz / 30 % (a margin beyond the envelope: 14 ms gaps) |

Every scenario starts at power-on with the gate closed. The assertions are the `gate: state=…` lines of the device console and the levels of the chip outputs (`expect-pin`). An extra pulse changes the motion of a Nice gate, so the position at a limit or halfway after a pause shows that there were no extra pulses. The exception is the one second of reversal after an obstacle: the model ignores a step during it, and only the closure count catches it (scenarios 10 and 11). Every pulse also shows in the log as the chip line `contact released after … ms, closure N`.

## Running

1. Build the firmware (in `firmware`):

   ```bash
   source ~/esp/esp-idf/export.sh && source ~/esp/esp-matter/export.sh
   export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.sim"
   idf.py -B build-s3 -D SDKCONFIG=build-s3/sdkconfig set-target esp32s3
   idf.py -B build-s3 -D SDKCONFIG=build-s3/sdkconfig build
   ```

2. Build the chip, check the circuit and run all scenarios (in `virtual/wokwi`). The script needs a Wokwi CLI token in `WOKWI_CLI_TOKEN`:

   ```bash
   WOKWI_CLI_TOKEN="$(cat ~/.config/wokwi/token)" ./run-scenarios.sh
   ```

   To run only some scenarios, name them: `./run-scenarios.sh 01-open-from-closed chop-open-close`. Logs go to `out/`. The token is read only from the environment and is never written anywhere.

3. One scenario by hand:

   ```bash
   C_INCLUDE_PATH=../../emulator/components/nice_core/include:../../shared/include \
       ~/.wokwi/bin/wokwi-cli chip compile nice-sna2.chip.c ../../emulator/components/nice_core/nice_sim.c -o nice-sna2.chip.wasm
   WOKWI_CLI_TOKEN="$(cat ~/.config/wokwi/token)" ~/.wokwi/bin/wokwi-cli . --timeout 120000 --scenario scenarios/01-open-from-closed.yaml
   ```

   Without `--scenario` and with `--interactive` you can type commands into the console yourself. In VS Code with the Wokwi extension this directory opens as a project: the buttons, the LEDs and the turning stepper are visible.

The first chip build downloads `wokwi-api.h`. Git ignores it, the `*.wasm` files and `out/`.
