#include "gate_status.h"

bool gate_status_publish_due(const gate_status_t *published, const gate_status_t *now, bool command_taken)
{
    return command_taken || published->state != now->state || published->busy != now->busy ||
           published->error != now->error;
}

const char *gate_state_name(gate_state_t state)
{
    switch (state) {
    case GATE_STATE_UNKNOWN: return "UNKNOWN";
    case GATE_STATE_CLOSED: return "CLOSED";
    case GATE_STATE_NOT_CLOSED: return "NOT_CLOSED";
    case GATE_STATE_OPENING: return "OPENING";
    case GATE_STATE_CLOSING: return "CLOSING";
    }
    return "?";
}

const char *gate_error_name(gate_error_t error)
{
    switch (error) {
    case GATE_ERR_NONE: return "NONE";
    case GATE_ERR_UNKNOWN_STATE: return "UNKNOWN_STATE";
    case GATE_ERR_NO_CONFIRM: return "NO_CONFIRM";
    case GATE_ERR_WRONG_DIRECTION: return "WRONG_DIRECTION";
    case GATE_ERR_OBSTACLE: return "OBSTACLE";
    case GATE_ERR_UNEXPECTED_STOP: return "UNEXPECTED_STOP";
    case GATE_ERR_TRAVEL_TIMEOUT: return "TRAVEL_TIMEOUT";
    }
    return "?";
}
