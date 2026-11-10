/* Pure logic of the bench emulator: console argument parsing, input debounce and the A4988 step generator.
 * No ESP-IDF dependency, so it is covered by the host tests (firmware/test_host/test_emu_logic.c). */
#ifndef EMU_LOGIC_H
#define EMU_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Travel time bounds for the `travel` command. The lower bound keeps the step rate under 2 kHz. */
#define EMU_TRAVEL_MIN_MS 5000u
#define EMU_TRAVEL_MAX_MS 60000u
/* Chop frequency bound for the `chop` command. The model works in whole milliseconds, so 100 Hz (10 ms period)
 * is the highest rate where the duty still has a usable resolution. */
#define EMU_CHOP_MAX_HZ 100u

/* Full travel in motor steps: 3 turns of a 200-step motor at 1/16 microstepping (3 * 200 * 16).
 * 15 s travel gives 640 steps/s; the shortest allowed travel (5 s) gives 1920 steps/s, under the 2 kHz limit
 * of a 17HS4023 at 0.7 A. */
#define EMU_STEPS_PER_TRAVEL 9600

/* Step generator period. One step needs two ticks (pulse high, pulse low), so the maximum rate is 2 kHz. */
#define EMU_STEPPER_TICK_US 250u
/* The driver stays enabled this long after the last step, so a slow move does not toggle ENABLE. */
#define EMU_STEPPER_HOLD_TICKS 400u

/* Number of consecutive 1 ms samples that must agree before an input change is accepted. */
#define EMU_DEBOUNCE_SAMPLES 5u

/* Strict parsers: plain decimal digits only, the whole string must be consumed, range checked.
 * On failure the outputs are left unchanged. */
bool emu_parse_travel(const char *text, uint32_t *travel_ms);
/* hz "0" means steady FLASH: duty is then optional (may be NULL) and ignored. Otherwise duty is 1..99. */
bool emu_parse_chop(const char *hz, const char *duty, uint16_t *chop_hz, uint8_t *duty_pct);

/* Chop period and on-time in whole milliseconds as nice_sim produces them (period rounds down, on-time rounds up). */
void emu_chop_effective(uint16_t chop_hz, uint8_t duty_pct, uint32_t *period_ms, uint32_t *on_ms);

/* Whether chop/travel may re-initialise the model. Re-init forgets the contact state, so it is refused while PP is
 * held (raw or debounced), even if the model is CLOSED (the gate can close while the contact is still held). */
typedef enum { EMU_RECONFIG_OK, EMU_RECONFIG_NOT_CLOSED, EMU_RECONFIG_CONTACT_HELD } emu_reconfig_t;
emu_reconfig_t emu_reconfigure_check(bool model_closed, bool raw_contact_closed, bool debounced_contact_closed);

/* Measurement history of the bench. It lives outside the model, so a re-init never resets it. */
typedef struct {
    uint32_t closures;  /* debounced PP closures, from the relay and from the wall button alike */
    uint32_t last_closure_ms;
    bool always_open_seen;
} emu_history_t;

void emu_history_init(emu_history_t *h);
void emu_history_record_closure(emu_history_t *h, uint32_t length_ms);
void emu_history_note_always_open(emu_history_t *h, bool model_flag);

/* Motor step target for a gate position in permille (clamped to 0..1000). */
int32_t emu_steps_for_permille(int32_t permille);

typedef struct {
    bool stable;
    uint8_t disagree_count;
} emu_debounce_t;

void emu_debounce_init(emu_debounce_t *d, bool initial);
/* Feeds one raw sample. Returns true on the sample that makes a new level stable. Both edges are delayed by the
 * same number of samples, so the length of a pulse is preserved up to the bounce time. */
bool emu_debounce_update(emu_debounce_t *d, bool raw);
bool emu_debounce_stable(const emu_debounce_t *d);

/* Raw PP observer next to the debounce. The debounce hides closed periods shorter than EMU_DEBOUNCE_SAMPLES and
 * chatter; this reports them so that "no closure" on the bench also proves "no short closure".
 * An episode is the raw closed time collected while the debounced level is open, from the first closed sample until
 * the raw input has been open for EMU_DEBOUNCE_SAMPLES samples. If the debounce accepts a closure meanwhile, the
 * episode belongs to that closure (leading bounce) and is dropped. Bounce on release comes before the debounced
 * level falls, so it is not counted either. */
typedef struct {
    uint32_t pending_ms;
    uint32_t open_run;
    uint32_t count;
} emu_glitch_t;

void emu_glitch_init(emu_glitch_t *g);
/* Call once per 1 ms sample, after emu_debounce_update. Returns the closed milliseconds of a finished glitch
 * episode (and counts it), otherwise 0. */
uint32_t emu_glitch_update(emu_glitch_t *g, bool raw_closed, bool debounced_closed);
uint32_t emu_glitch_count(const emu_glitch_t *g);

typedef struct {
    int32_t position;
    bool step_high;
    bool dir_open;
    bool enabled;
    uint32_t idle_ticks;
} emu_stepper_t;

typedef struct {
    bool step;      /* STEP pin level */
    bool dir_open;  /* DIR pin level: high = opening */
    bool enable;    /* driver enabled (the ENABLE pin is active low) */
} emu_stepper_out_t;

void emu_stepper_init(emu_stepper_t *st);
/* One generator tick (EMU_STEPPER_TICK_US). Enabling the driver and changing DIR each take a tick of their own,
 * so the A4988 setup times are met; a step is a one-tick pulse. */
emu_stepper_out_t emu_stepper_tick(emu_stepper_t *st, int32_t target);

#ifdef __cplusplus
}
#endif

#endif /* EMU_LOGIC_H */
