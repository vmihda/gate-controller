/* Spacing of relay pulses by their real start times, the last guard of the 1 s pause between pulses.
 * The controller spaces the pulses it asks for, but stamps them before the RMT starts them; relay_out
 * stamps the real start with its own clock and refuses a pulse that would come too early.
 * Pure C, no heap, no ESP-IDF dependency. Times are uint32_t milliseconds and may wrap; they are only
 * compared by subtraction. Not thread-safe: one caller at a time. */
#ifndef PULSE_SPACING_H
#define PULSE_SPACING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fields are private; the struct is public only so the caller can own it without a heap. */
typedef struct pulse_spacing {
    bool any;               /* a pulse was started */
    uint32_t last_start_ms;
} pulse_spacing_t;

void pulse_spacing_init(pulse_spacing_t *s);
/* True if a pulse starting at now_ms is at least GATE_PULSE_SPACING_MS after the last start. Read now_ms
 * right before the start. After 49.7 days without a pulse the subtraction can alias a recent start once:
 * then a pulse is refused, never let through early. */
bool pulse_spacing_allows(const pulse_spacing_t *s, uint32_t now_ms);
/* Records a started pulse. Read start_ms once the start is done, so it is never earlier than the real one. */
void pulse_spacing_started(pulse_spacing_t *s, uint32_t start_ms);

#ifdef __cplusplus
}
#endif

#endif /* PULSE_SPACING_H */
