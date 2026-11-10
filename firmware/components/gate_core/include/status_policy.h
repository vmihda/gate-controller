// Picks the status LED pattern from device state. Pure function, no hardware access.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Declared in priority order: the first pattern that applies wins.
typedef enum {
    LED_RESETTING,
    LED_COMMAND_ERROR,
    LED_NO_WIFI,
    LED_AWAITING_COMMISSIONING,
    LED_STARTING,
    LED_OK
} led_pattern_t;

typedef struct {
    bool resetting;
    bool command_error;
    bool commissioned;
    bool wifi_connected;
    bool started;
} status_inputs_t;

// Returns the highest-priority pattern that applies. The "starting" pattern ranks below
// "awaiting commissioning", so an uncommissioned device shows the latter even before `started`.
led_pattern_t status_policy_select(const status_inputs_t *in);

// The `started` input: true from the first recognized gate state on. The decoder reports UNKNOWN at the
// start and end of every motion, which must not bring the "starting" pattern back.
bool status_policy_started(bool started_before, bool state_recognized);

// Whether the LED is lit for `pattern` at `now_ms`. Periodic in time, so the uint32_t wrap only shifts the phase.
bool status_policy_led_on(led_pattern_t pattern, uint32_t now_ms);

// Whether the LED is lit at `now_ms`, with a short BOOT press shown until `show_until_ms` (0: no press).
// The press lights a healthy device (LED_OK, whose short flash every 30 s is easy to miss) steadily; every
// other pattern already shows the state and stays as it is. Both times are the 64-bit uptime, so the
// deadline never wraps.
bool status_policy_led_lit(led_pattern_t pattern, uint64_t now_ms, uint64_t show_until_ms);

#ifdef __cplusplus
}
#endif
