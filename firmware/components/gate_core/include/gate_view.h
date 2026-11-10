/* What Home is told about the gate, from the status the gate task publishes. Pure C, no heap, no ESP-IDF
 * dependency; matter_app maps the view onto the attributes of the selected Matter device type. */
#ifndef GATE_VIEW_H
#define GATE_VIEW_H

#include <stdbool.h>

#include "flash_decoder.h"
#include "gate_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { GATE_POSITION_UNKNOWN, GATE_POSITION_CLOSED, GATE_POSITION_OPEN } gate_position_t;

typedef struct gate_view {
    gate_position_t position;  /* last stationary state: CLOSED -> closed, NOT_CLOSED -> open; UNKNOWN until the first */
    gate_motion_t motion;      /* GATE_MOTION_NONE while stationary */
    bool busy;                 /* a command is running */
    gate_error_t error;        /* result of the last finished command, cleared when the next one starts */
} gate_view_t;

void gate_view_init(gate_view_t *v);
/* Folds in one published status. UNKNOWN keeps the last position and motion, so Home does not flap at the
 * start and the end of a blinking run. */
void gate_view_update(gate_view_t *v, gate_state_t state, bool busy, gate_error_t error);
/* The gate moves or a command runs. */
bool gate_view_active(const gate_view_t *v);
/* Where the gate is heading: the motion's direction, else the position. False while a command waits for the
 * gate to move: the target the command set stays. */
bool gate_view_target(const gate_view_t *v, gate_position_t *target);
/* For a display with only "shut" and "open" (the Door Lock candidate): shut only while closed and still,
 * open while moving or open. False while nothing is known yet: leave the display as it is. */
bool gate_view_shut(const gate_view_t *v, bool *shut);

#ifdef __cplusplus
}
#endif

#endif /* GATE_VIEW_H */
