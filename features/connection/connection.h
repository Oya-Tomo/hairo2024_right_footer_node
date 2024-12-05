#pragma once

#include "pico/stdlib.h"
#include "pico/i2c_slave.h"
#include "hardware/i2c.h"

#include "features/connection/context.h"

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event);

void connection_setup();

system_state_t get_system_state();

drive_state_t get_drive_state();

flipper_state_t get_flipper_state();
