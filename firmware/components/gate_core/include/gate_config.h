/* Timing constants. Signal-derived values come from flash_timing.h: change the numbers there, not here. */
#ifndef GATE_CONFIG_H
#define GATE_CONFIG_H

#include "flash_timing.h"

/* Accepted blink period bands, bounds inclusive. The lit and dark halves of a blink are held to the
 * same tolerance around their nominal share of the period (FLASH_BLINK_DUTY_PCT). */
#define GATE_PERIOD_MIN_MS(nominal_ms) (((nominal_ms) * (100u - FLASH_PERIOD_TOLERANCE_PCT)) / 100u)
#define GATE_PERIOD_MAX_MS(nominal_ms) (((nominal_ms) * (100u + FLASH_PERIOD_TOLERANCE_PCT)) / 100u)

/* Chopping moves a decoded edge by at most one swallowed low gap: a rise comes up to FLASH_GLITCH_MS
 * late, a fall up to FLASH_GLITCH_MS early. So a period (same-direction edges) is measured within
 * +-GATE_EDGE_JITTER_MS and a half (opposite edges) within +-2 x GATE_EDGE_JITTER_MS; the bands are
 * widened by that much. */
#define GATE_EDGE_JITTER_MS FLASH_GLITCH_MS

/*
 * Signal that stays unchanged longer than this is steady (open or closed):
 * 1.5 x the longest half-period including tolerance, rounded up.
 *   GATE_STEADY_AFTER_MS = ceil(3 * SLOW_PERIOD * DUTY% * (100 + TOL%) / (2 * 100 * 100))
 * Bench values: ceil(3 * 1000 * 50 * 125 / 20000) = ceil(937.5) = 938 ms.
 * The slow period is the longest, so its longer phase (the duty or its complement) bounds it.
 */
#define GATE_LONGEST_PHASE_PCT (FLASH_BLINK_DUTY_PCT > (100u - FLASH_BLINK_DUTY_PCT) \
                                    ? FLASH_BLINK_DUTY_PCT : (100u - FLASH_BLINK_DUTY_PCT))
#define GATE_STEADY_AFTER_MS                                                          \
    ((3u * FLASH_SLOW_PERIOD_MS * GATE_LONGEST_PHASE_PCT * (100u + FLASH_PERIOD_TOLERANCE_PCT) + 19999u) / 20000u)

/* Period of the decoder tick and the controller step (the gate task loop). */
#define GATE_STEP_MS 10u

/*
 * A motion reading is fresh while the decoder's last edge is at most this old (Ruling S). A moving gate
 * blinks, so its decoded edges come at most one half apart; a stopped gate is steadily lit and has none.
 * Per blink period: the longest half including tolerance, plus 2 x GATE_EDGE_JITTER_MS (both edges of
 * the half can move), plus FLASH_GLITCH_MS and one step (a fall is decoded only once the glitch filter
 * has passed, at the next step). So a gate still blinking at that period always qualifies.
 * Bench values: slow 625 + 30 + 15 + 10 = 680 ms, fast 250 + 30 + 15 + 10 = 305 ms.
 * It is taken per blink type: one window sized for the slow blink would call a closing gate moving for
 * up to 680 ms after it stopped, in a lit half that is only 250 ms long.
 */
#define GATE_FRESH_EDGE_MS(period_ms)                                                                    \
    (GATE_PERIOD_MAX_MS((period_ms) * GATE_LONGEST_PHASE_PCT / 100u) + 2u * GATE_EDGE_JITTER_MS +       \
     FLASH_GLITCH_MS + GATE_STEP_MS)

/* Command handling, see the commands section of the spec. */
#define GATE_PULSE_MS 500u            /* relay pulse length */
#define GATE_PULSE_GAP_MS 1000u       /* minimum pause from the end of one pulse to the start of the next */
#define GATE_PULSE_SPACING_MS (GATE_PULSE_MS + GATE_PULSE_GAP_MS)  /* shortest start to start, held by relay_out */
/* relay_out stamps a pulse with its own clock, read after the controller's step. The controller asks for
 * the next pulse this much later than GATE_PULSE_SPACING_MS, so relay_out does not refuse it over that
 * delay; only a task stalled longer than this gets a pulse refused (the command then fails). */
#define GATE_PULSE_SPACING_MARGIN_MS 20u
#define GATE_MAX_PULSES 3u            /* per command */
#define GATE_UNKNOWN_WAIT_MS (2u * FLASH_SLOW_PERIOD_MS)  /* longest wait for a recognized state */
#define GATE_CONFIRM_TIMEOUT_MS 3000u /* expected reaction to a pulse, counted from the pulse end */
#define GATE_MAX_TRAVEL_MS 60000u     /* longest recognized motion */

/* Device LED and BOOT button, see the device LED and button section of the spec. */
#define GATE_BUTTON_DEBOUNCE_MS 40u      /* shorter presses are contact bounce */
#define GATE_BUTTON_LONG_PRESS_MS 5000u  /* Matter factory reset */
#define GATE_LED_SHOW_MS 3000u           /* a short press shows the state this long */

#endif /* GATE_CONFIG_H */
