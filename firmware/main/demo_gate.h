// GATE_DEMO: the gate is simulated inside the firmware by the Nice model (nice_sim), for the Apple Home
// check without the bench. demo_gate.cpp implements the flash_in.h and relay_out.h interfaces on that model
// and is built instead of flash_in.cpp and relay_out.cpp, so gate_task runs the same FLASH decoder and
// command controller as on the bench. No GPIO is configured for the relay (D1) or FLASH (D3).
#pragma once

// The wall button: one Step-by-step contact closure of the Nice model. The short BOOT press in the demo build.
// A closure that is still held (a relay pulse) absorbs the press, as a button wired in parallel would.
void demo_gate_press_wall_button();
