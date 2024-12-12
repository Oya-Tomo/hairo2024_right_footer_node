#pragma once

#include "pico/stdlib.h"
#include "pico/i2c_slave.h"
#include "hardware/i2c.h"

// I2C context

typedef struct
{
    uint8_t memory[256];
    uint8_t address;
    bool written_address;
} i2c_context_t;

// System State Structures

const uint8_t IS_RUNNING_BYTE = 0x00;

const uint8_t SPEED_BYTE = 0x01;

const uint8_t FRONT_ANGLE_BYTE = 0x02;
const uint8_t BACK_ANGLE_BYTE = 0x03;

typedef struct
{
    bool is_running;
} system_state_t;

typedef struct
{
    float speed; // -1.0 ~ 1.0
} drive_state_t;

typedef struct
{
    float front_angle; // degree (-PI/2 ~ PI/2)
    float back_angle;  // degree (-PI/2 ~ PI/2)
} flipper_state_t;

system_state_t convert_to_system_state_t(uint8_t *data);

drive_state_t convert_to_drive_state_t(uint8_t *data);

flipper_state_t convert_to_flipper_state_t(uint8_t *data);
