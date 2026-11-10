/* Classifies BOOT button presses into a short press and a long press. Pure C, no ESP-IDF dependency.
 * Times are uint32_t milliseconds and may wrap; they are only compared by subtraction. */
#ifndef BUTTON_PRESS_H
#define BUTTON_PRESS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { BUTTON_EVENT_NONE, BUTTON_EVENT_SHORT, BUTTON_EVENT_LONG } button_event_t;

/* Private to button_press.c; public only so the caller can own the struct without a heap. */
typedef struct button_press {
    bool armed;         /* a debounced release was seen since init: only then can a press count */
    bool release_seen;  /* released samples are being timed */
    uint32_t released_since_ms;
    bool down;
    bool long_reported;
    uint32_t down_since_ms;  /* first pressed sample */
    uint32_t held_ms;        /* from the first to the latest pressed sample */
} button_press_t;

void button_press_init(button_press_t *b);

/* Call with every sample of the button, pressed or not. The hold time runs from the first to the latest
 * pressed sample. A hold of GATE_BUTTON_LONG_PRESS_MS gives BUTTON_EVENT_LONG once, while the button is still
 * down, and nothing at its release. A shorter hold of at least GATE_BUTTON_DEBOUNCE_MS gives
 * BUTTON_EVENT_SHORT at the release. A button that is already pressed when sampling starts (held at boot) is
 * ignored until it has been released for GATE_BUTTON_DEBOUNCE_MS, so a stuck-low button never gives an event. */
button_event_t button_press_sample(button_press_t *b, bool pressed, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_PRESS_H */
