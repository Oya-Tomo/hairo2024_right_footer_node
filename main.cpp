#include <stdio.h>
#include "pico/stdlib.h"

#include "features/connection/connection.h"
#include "features/drive/drive.h"
#include "features/flipper/flipper.h"
#include "features/ticker/ticker.h"

void irq_callback_register(uint gpio, uint32_t events)
{
    drive_callback_register(gpio, events);
    flipper_callback_register(gpio, events);
}

void setup()
{
    stdio_init_all();

    connection_setup();
    drive_setup();
    flipper_setup();

    gpio_set_irq_callback(irq_callback_register);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void task()
{
    system_state_t system_state = get_system_state();

    if (system_state.is_running)
    {
        drive_task();
        flipper_task();
    }
    else
    {
        drive_stop();
    }
}

int main()
{
    setup();

    while (true)
    {
        task();
        frame_wait();
    }

    return 0;
}
