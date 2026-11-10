// FLASH input on D3 from the PC817 optocoupler: low while the Nice FLASH output is lit.
// Every level change is queued from the GPIO interrupt with its esp_timer time.
#pragma once

#include <cstdint>

#include <esp_err.h>

struct flash_edge_t {
    bool lit;
    uint32_t at_ms;  // esp_timer time; wraps after about 49.7 days like the gate_core times
};

[[nodiscard]] esp_err_t flash_in_init();

// The level now, to seed the decoder.
[[nodiscard]] bool flash_in_is_lit();

// Takes the oldest queued edge without waiting. False when the queue is empty.
[[nodiscard]] bool flash_in_take_edge(flash_edge_t *edge);

// True once after edges were dropped because the queue was full; the caller then re-reads the level.
[[nodiscard]] bool flash_in_take_overflow();

// Drops queued edges, before the decoder is seeded.
void flash_in_discard_edges();
