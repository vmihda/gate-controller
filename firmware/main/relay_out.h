// Relay output on D1. This is the only module that touches D1, and the pulse is timed by the RMT
// hardware: once started it ends on its own, even if the CPU stops. It also owns the watchdog setup
// for the task that drives the relay.
#pragma once

#include <cstdint>

#include <esp_err.h>

// Call first in app_main: D1 becomes an output at low level, then the RMT channel takes it over.
// Blocks for 1 s before the handover (a pulse left over from before a software restart must be
// over first, see relay_out.cpp), so no pulse can be requested earlier. On an error D1 stays a low
// output.
[[nodiscard]] esp_err_t relay_out_init();

// Call from the task that drives the relay: subscribes it to the task watchdog and arms the RTC
// watchdog. From then on that task must call relay_out_feed_watchdogs() at least every 1 s.
[[nodiscard]] esp_err_t relay_out_watch_this_task();
void relay_out_feed_watchdogs();

// Starts one high pulse of `ms` on D1. False if a pulse is still active, `ms` is out of range, the
// previous accepted pulse started less than GATE_PULSE_MS + GATE_PULSE_GAP_MS ago (pulse_spacing), or
// the RMT driver refused it.
[[nodiscard]] bool relay_out_pulse(uint32_t ms);
