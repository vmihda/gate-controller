#!/usr/bin/env bash
# Compiles the Nice SNA2 chip, lints the diagram and runs the Wokwi scenarios against the GATE_SIM
# firmware in ../../firmware/build-s3 (build it first, see README.md).
# Needs WOKWI_CLI_TOKEN in the environment; this script never reads or stores the token.
# Usage: run-scenarios.sh [scenario-name ...]   (default: all of scenarios/*.yaml)
set -euo pipefail

cd "$(dirname "$0")"
WOKWI_CLI="${WOKWI_CLI:-$HOME/.wokwi/bin/wokwi-cli}"
TIMEOUT_MS="${TIMEOUT_MS:-120000}"  # simulated time per scenario
OUT=out
# Chopped FLASH variants: chop_hz and chop_duty of the chip. 100 Hz is rectified 50 Hz mains;
# 50 Hz at 50 % (10 ms gaps) is the corner of the spec envelope; 50 Hz at 30 % (14 ms gaps) is a margin
# beyond the spec envelope.
CHOP_VARIANTS=("100 30" "50 50" "50 30")

: "${WOKWI_CLI_TOKEN:?set WOKWI_CLI_TOKEN, see README.md}"
[[ -f ../../firmware/build-s3/flasher_args.json ]] || { echo "firmware/build-s3 is missing, see README.md" >&2; exit 1; }

mkdir -p "$OUT"
C_INCLUDE_PATH=../../emulator/components/nice_core/include:../../shared/include \
    "$WOKWI_CLI" chip compile nice-sna2.chip.c ../../emulator/components/nice_core/nice_sim.c \
    -o nice-sna2.chip.wasm > "$OUT/chip-compile.log" 2>&1 || { cat "$OUT/chip-compile.log" >&2; exit 1; }
"$WOKWI_CLI" lint --warnings-as-errors diagram.json

run() {  # run <scenario file> <diagram file> <log name>
    local status=PASS
    "$WOKWI_CLI" . --quiet --timeout "$TIMEOUT_MS" --scenario "$1" --diagram-file "$2" \
        --serial-log-file "$OUT/$3.serial.log" > "$OUT/$3.log" 2>&1 || status=FAIL
    printf '%s %s\n' "$status" "$3"
    [[ $status == PASS ]]
}

if [[ $# -gt 0 ]]; then
    scenarios=("${@/#/scenarios/}")
    scenarios=("${scenarios[@]/%/.yaml}")
else
    scenarios=(scenarios/*.yaml)
fi

failed=0
for scenario in "${scenarios[@]}"; do
    name=$(basename "$scenario" .yaml)
    if [[ $name == chop-* ]]; then
        for variant in "${CHOP_VARIANTS[@]}"; do
            read -r hz duty <<< "$variant"
            diagram="$OUT/diagram-chop-${hz}hz-${duty}.json"
            sed -e "s/\"chop_hz\": \"0\"/\"chop_hz\": \"$hz\"/" \
                -e "s/\"chop_duty\": \"30\"/\"chop_duty\": \"$duty\"/" diagram.json > "$diagram"
            run "$scenario" "$diagram" "$name-${hz}hz-${duty}" || failed=$((failed + 1))
        done
    else
        run "$scenario" diagram.json "$name" || failed=$((failed + 1))
    fi
done
echo "failed: $failed"
[[ $failed -eq 0 ]]
