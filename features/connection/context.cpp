#include "features/connection/context.h"

#include <cmath>

system_state_t convert_to_system_state_t(uint8_t *data)
{
    system_state_t state;
    state.is_running = (bool)data[IS_RUNNING_BYTE];
    return state;
}

drive_state_t convert_to_drive_state_t(uint8_t *data)
{
    drive_state_t state;
    state.speed = (float)((int8_t)data[SPEED_BYTE]) / 128.0;
    return state;
}

flipper_state_t convert_to_flipper_state_t(uint8_t *data)
{
    flipper_state_t state;
    state.front_angle = (float)((int8_t)data[FRONT_ANGLE_BYTE]) * (M_PI / 2.0) / 128.0;
    state.back_angle = (float)((int8_t)data[BACK_ANGLE_BYTE]) * (M_PI / 2.0) / 128.0;
    return state;
}
