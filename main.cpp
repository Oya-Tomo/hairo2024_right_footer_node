#include "pico/stdlib.h"

#include "features/connection/connection.h"
#include "features/drive/drive.h"
#include "features/flipper/flipper.h"
#include "features/ticker/ticker.h"

#define DEBUG

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

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
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

#ifdef DEBUG
    drive_state_t drive_state = get_drive_state();
    flipper_state_t flipper_state = get_flipper_state();

    stdio_printf("Drive: %f | ", drive_state.speed);
    stdio_printf("Flipper: %f %f\n", flipper_state.front_angle, flipper_state.back_angle);
#endif
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
