# Nice SPIN garage gate controller over Matter

A Matter over Wi-Fi controller that adds a Nice SPIN garage gate (Nice SNA2 control board) to Apple Home. A relay on the board's Step-by-step input opens and closes the gate. The FLASH output, set to "gate open indicator" mode, gives the gate state. If the gate does not respond to a command, the device reports an error.

- Bench device: Seeed Studio XIAO ESP32-S3.
- Gate device: Seeed Studio XIAO ESP32-C6.
- Bench emulator of the Nice board: ESP32-WROOM-32 with an A4988 driver and a 17HS4023 stepper motor.

## Layout

| Path | What it is |
|---|---|
| `firmware/` | Device firmware (ESP-IDF + esp-matter) |
| `firmware/components/gate_core/` | Hardware-independent gate logic: FLASH decoder, command controller, status policy |
| `firmware/test_host/` | Host unit tests of `gate_core` and of the Nice model |
| `emulator/` | Firmware of the Nice SNA2 emulator for the bench |
| `emulator/components/nice_core/` | Model of the Nice board, shared by the emulator, the host tests and Wokwi |
| `shared/include/` | FLASH timing shared by the device and the model |
| `virtual/wokwi/` | Virtual bench in Wokwi, see `virtual/wokwi/README.md` |

Pins: device in `firmware/main/board_config.h`, emulator in `emulator/main/emu_board.h`.

## Tools

ESP-IDF v6.0.2 and esp-matter v1.6.1 (branch `release/v1.6.1`). Every firmware build runs in the environment of:

```bash
source ~/esp/esp-idf/export.sh && source ~/esp/esp-matter/export.sh
```

## Host tests

They build with the host C compiler and CMake, without ESP-IDF:

```bash
cmake -S firmware/test_host -B build-host && cmake --build build-host && ctest --test-dir build-host
```

## Build and flash

Run in `firmware`.

Each board has its own Home pairing code. Copy `firmware/units/unit.defaults.example` to `firmware/units/<unit>.defaults` and fill in the passcode and the discriminator as the file explains. Git ignores these copies. At startup the board prints the code to the console in the `Matter setup code`, `Manual pairing code` and `QR code` lines.

XIAO ESP32-C6 at the gate:

```bash
export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.matter;units/gate-c6.defaults"
idf.py -B build-c6-matter -D SDKCONFIG=build-c6-matter/sdkconfig set-target esp32c6
idf.py -B build-c6-matter -D SDKCONFIG=build-c6-matter/sdkconfig -p PORT build erase-flash flash monitor
```

XIAO ESP32-S3 on the bench: the same commands with `units/bench-s3.defaults`, the `build-s3-matter` directory and `set-target esp32s3`.

- Use a separate build directory for each set of settings. On first configuration a directory records whether the build includes Matter, and a build without Matter will not build there afterwards. That record does not tell the working build from the demo build, so always give the demo its own directory.
- Run `set-target` only for a new directory, because it recreates the build settings.
- Use `erase-flash` only the first time you flash a board. It wipes the whole flash, Home pairing included.
- The gate shows up in Home as a Closure by default. Alternatives: add `sdkconfig.window-covering` or `sdkconfig.door-lock` after `sdkconfig.matter`.
- `sdkconfig.debug` adds full logs and error names.

**Never put `sdkconfig.demo` on the board at the gate.** The demo build has no relay and cannot control a real gate. It simulates the Nice board inside the firmware, so you can see how Home shows the gate without the bench. `sdkconfig.sim` is for Wokwi only.

## Emulator

Run in `emulator`:

```bash
idf.py set-target esp32 && idf.py -p PORT build flash monitor
```
