/* The status the gate task publishes, and when it publishes it. Pure C, no ESP-IDF dependency. */
#ifndef GATE_STATUS_H
#define GATE_STATUS_H

#include <stdbool.h>

#include "gate_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gate_status {
    gate_state_t state;
    bool busy;
    gate_error_t error;
} gate_status_t;

/* True when the listener must be called: the status changed, or a command was taken. A command the gate
 * already satisfies ends in the same step with the status unchanged, and Home still waits for an answer:
 * after a Closure MoveTo or a Window Covering target write it shows motion until the endpoint is
 * written again. */
bool gate_status_publish_due(const gate_status_t *published, const gate_status_t *now, bool command_taken);

/* Names for the log, such as "NOT_CLOSED" and "OBSTACLE". */
const char *gate_state_name(gate_state_t state);
const char *gate_error_name(gate_error_t error);

#ifdef __cplusplus
}
#endif

#endif /* GATE_STATUS_H */
