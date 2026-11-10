#include "button_press.h"

#include "gate_config.h"

void button_press_init(button_press_t *b)
{
    *b = (button_press_t){0};
}

button_event_t button_press_sample(button_press_t *b, bool pressed, uint32_t now_ms)
{
    if (!pressed) {
        if (!b->armed) {
            if (!b->release_seen) {
                b->release_seen = true;
                b->released_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - b->released_since_ms) >= GATE_BUTTON_DEBOUNCE_MS) {
                b->armed = true;
            }
        }
        const bool was_short = b->down && !b->long_reported && b->held_ms >= GATE_BUTTON_DEBOUNCE_MS;
        b->down = false;
        return was_short ? BUTTON_EVENT_SHORT : BUTTON_EVENT_NONE;
    }
    if (!b->armed) {
        b->release_seen = false;  /* held at power-up, or bouncing: not a press */
        return BUTTON_EVENT_NONE;
    }
    if (!b->down) {
        b->down = true;
        b->long_reported = false;
        b->down_since_ms = now_ms;
        b->held_ms = 0;
        return BUTTON_EVENT_NONE;
    }
    b->held_ms = (uint32_t)(now_ms - b->down_since_ms);
    if (!b->long_reported && b->held_ms >= GATE_BUTTON_LONG_PRESS_MS) {
        b->long_reported = true;
        return BUTTON_EVENT_LONG;
    }
    return BUTTON_EVENT_NONE;
}
