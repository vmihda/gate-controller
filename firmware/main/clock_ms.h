// Millisecond clocks from esp_timer.
#pragma once

#include <cstdint>

#include <esp_timer.h>

// Never wraps in practice; use it for deadlines.
inline uint64_t uptime_ms()
{
    return static_cast<uint64_t>(esp_timer_get_time() / 1000);
}

// The clock of all gate_core times. uint32_t wraps after about 49.7 days; gate_core compares these
// times only by subtraction, so never compare them as deadlines.
inline uint32_t clock_ms()
{
    return static_cast<uint32_t>(uptime_ms());
}
