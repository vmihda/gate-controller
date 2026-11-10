#include "status_policy.h"

/* Pattern timing from the spec LED table. "Dim" for LED_OK is a short flash: the LED is on/off only. */
#define RESETTING_HALF_PERIOD_MS 50u    /* very fast blink */
#define ERROR_HALF_PERIOD_MS 250u       /* fast blink */
#define AWAITING_HALF_PERIOD_MS 500u    /* once per second */
#define NO_WIFI_PERIOD_MS 2000u         /* two short flashes every 2 s */
#define NO_WIFI_FLASH_MS 100u
#define NO_WIFI_SECOND_FLASH_MS 300u    /* start of the second flash */
#define OK_PERIOD_MS 30000u             /* short flash every 30 s */
#define OK_FLASH_MS 50u

static bool square_wave(uint32_t now_ms, uint32_t half_period_ms) {
    return (now_ms / half_period_ms) % 2u == 0u;
}

led_pattern_t status_policy_select(const status_inputs_t *in) {
    if (in->resetting) {
        return LED_RESETTING;
    }
    if (in->command_error) {
        return LED_COMMAND_ERROR;
    }
    if (!in->commissioned) {
        return LED_AWAITING_COMMISSIONING;
    }
    if (!in->wifi_connected) {
        return LED_NO_WIFI;
    }
    if (!in->started) {
        return LED_STARTING;
    }
    return LED_OK;
}

bool status_policy_started(bool started_before, bool state_recognized) {
    return started_before || state_recognized;
}

bool status_policy_led_on(led_pattern_t pattern, uint32_t now_ms) {
    switch (pattern) {
    case LED_RESETTING:
        return square_wave(now_ms, RESETTING_HALF_PERIOD_MS);
    case LED_COMMAND_ERROR:
        return square_wave(now_ms, ERROR_HALF_PERIOD_MS);
    case LED_NO_WIFI: {
        const uint32_t phase = now_ms % NO_WIFI_PERIOD_MS;
        return phase < NO_WIFI_FLASH_MS ||
               (phase >= NO_WIFI_SECOND_FLASH_MS && phase < NO_WIFI_SECOND_FLASH_MS + NO_WIFI_FLASH_MS);
    }
    case LED_AWAITING_COMMISSIONING:
        return square_wave(now_ms, AWAITING_HALF_PERIOD_MS);
    case LED_STARTING:
        return true;
    case LED_OK:
        return now_ms % OK_PERIOD_MS < OK_FLASH_MS;
    }
    return false;
}

bool status_policy_led_lit(led_pattern_t pattern, uint64_t now_ms, uint64_t show_until_ms) {
    /* The pattern is periodic, so the 32-bit wrap of its time only shifts the phase. */
    return (pattern == LED_OK && now_ms < show_until_ms) || status_policy_led_on(pattern, (uint32_t)now_ms);
}
