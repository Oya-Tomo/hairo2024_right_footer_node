#pragma once

#include "pico/stdlib.h"

void drive_callback_register(uint gpio, uint32_t events);

void drive_setup();

void drive_task();

void drive_stop();

void set_center_belt_speed(double speed);

void set_front_belt_speed(double speed);

void set_back_belt_speed(double speed);