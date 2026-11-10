/* Bench emulator: pins and the interfaces between the small modules. */
#ifndef EMU_BOARD_H
#define EMU_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "emu_logic.h"
#include "nice_sim.h"

#define EMU_PIN_PP 27        /* Step-by-step input, external 10 kOhm pull-up, contact to GND = closed */
#define EMU_PIN_OBSTACLE 25  /* obstacle button to GND, internal pull-up */
#define EMU_PIN_FLASH 26     /* high = FLASH on (drives Q1 through R10) */
#define EMU_PIN_STEP 18      /* A4988 STEP */
#define EMU_PIN_DIR 19       /* A4988 DIR, high = opening */
#define EMU_PIN_ENABLE 23    /* A4988 ENABLE, active low, 10 kOhm pull-up on the board */

typedef struct {
    nice_sim_state_t state;
    int32_t position_permille;
    nice_sim_config_t cfg;
    uint32_t last_closure_ms;  /* history: survives chop/travel */
    bool always_open_seen;     /* history */
    uint32_t closures;         /* history: debounced PP closures, relay and wall button alike */
    uint32_t pp_edges;         /* raw PP edges counted by the GPIO ISR, independent of the debounce */
    uint32_t glitch_count;    /* raw PP closed episodes the debounce did not turn into a closure */
    uint32_t dropped_prints;  /* model lines lost because the print queue was full */
} emu_status_t;

/* pp_input.c: raw levels, debounced by the model task. */
void pp_input_init(void);
bool pp_input_contact_closed(void);
bool pp_input_obstacle_pressed(void);
/* Number of PP edges seen by a GPIO any-edge ISR (wraps at 2^32). Counts sub-millisecond pulses the 1 ms polling
 * cannot see. */
uint32_t pp_input_edge_count(void);

/* flash_out.c */
void flash_out_init(void);
void flash_out_set(bool on);

/* stepper.c: follows the target given in motor steps (0 = closed). */
void stepper_init(void);
void stepper_set_target(int32_t steps);

/* app_main.c: the model, shared with the console under a mutex. */
void emu_model_status(emu_status_t *out);
/* Re-initialises the model with cfg unless the model is not CLOSED or PP is held (see emu_reconfigure_check). */
emu_reconfig_t emu_model_reconfigure(const nice_sim_config_t *cfg);

/* emu_console.c */
void emu_console_start(void);
const char *emu_state_name(nice_sim_state_t state);

#endif /* EMU_BOARD_H */
