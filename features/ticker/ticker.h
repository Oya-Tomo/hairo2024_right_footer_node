#pragma once

#include "pico/stdlib.h"

int64_t break_frame_wait_callback(alarm_id_t id, void *user_data);

void frame_wait();

double get_frame_rate();

double get_dt();