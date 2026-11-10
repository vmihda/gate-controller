#include "pulse_spacing.h"

#include "gate_config.h"

void pulse_spacing_init(pulse_spacing_t *s)
{
    *s = (pulse_spacing_t){.any = false};
}

bool pulse_spacing_allows(const pulse_spacing_t *s, uint32_t now_ms)
{
    return !s->any || (uint32_t)(now_ms - s->last_start_ms) >= GATE_PULSE_SPACING_MS;
}

void pulse_spacing_started(pulse_spacing_t *s, uint32_t start_ms)
{
    s->any = true;
    s->last_start_ms = start_ms;
}
