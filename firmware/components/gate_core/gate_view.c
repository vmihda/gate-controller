#include "gate_view.h"

void gate_view_init(gate_view_t *v)
{
    *v = (gate_view_t){
        .position = GATE_POSITION_UNKNOWN,
        .motion = GATE_MOTION_NONE,
        .busy = false,
        .error = GATE_ERR_NONE,
    };
}

void gate_view_update(gate_view_t *v, gate_state_t state, bool busy, gate_error_t error)
{
    switch (state) {
    case GATE_STATE_CLOSED:
        v->position = GATE_POSITION_CLOSED;
        v->motion = GATE_MOTION_NONE;
        break;
    case GATE_STATE_NOT_CLOSED:
        v->position = GATE_POSITION_OPEN;
        v->motion = GATE_MOTION_NONE;
        break;
    case GATE_STATE_OPENING:
        v->motion = GATE_MOTION_OPENING;
        break;
    case GATE_STATE_CLOSING:
        v->motion = GATE_MOTION_CLOSING;
        break;
    case GATE_STATE_UNKNOWN:
        break;
    }
    v->busy = busy;
    v->error = error;
}

bool gate_view_active(const gate_view_t *v)
{
    return v->busy || v->motion != GATE_MOTION_NONE;
}

bool gate_view_target(const gate_view_t *v, gate_position_t *target)
{
    switch (v->motion) {
    case GATE_MOTION_OPENING:
        *target = GATE_POSITION_OPEN;
        return true;
    case GATE_MOTION_CLOSING:
        *target = GATE_POSITION_CLOSED;
        return true;
    case GATE_MOTION_NONE:
        break;
    }
    if (v->busy) {
        return false;
    }
    *target = v->position;
    return true;
}

bool gate_view_shut(const gate_view_t *v, bool *shut)
{
    if (v->motion == GATE_MOTION_NONE && v->position == GATE_POSITION_UNKNOWN) {
        return false;
    }
    *shut = v->motion == GATE_MOTION_NONE && v->position == GATE_POSITION_CLOSED;
    return true;
}
