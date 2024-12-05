#pragma once

#include "pico/stdlib.h"

void flipper_callback_register(uint gpio, uint32_t events);

void flipper_setup();

void flipper_task();

void set_front_flipper_angle(double angle);

void set_back_flipper_angle(double angle);