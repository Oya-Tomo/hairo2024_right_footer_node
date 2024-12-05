#include "features/connection/connection.h"

// I2C handler

static i2c_context_t context;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    switch (event)
    {
    case I2C_SLAVE_RECEIVE:
        if (!context.written_address)
        {
            context.address = i2c_read_byte_raw(i2c);
            context.written_address = true;
        }
        else
        {
            context.memory[context.address] = i2c_read_byte_raw(i2c);
            context.address++;
        }
        break;
    case I2C_SLAVE_REQUEST:
        i2c_write_byte_raw(i2c, context.memory[context.address]);
        context.address++;
        break;
    case I2C_SLAVE_FINISH:
        context.written_address = false;
        break;
    default:
        break;
    }
}

// I2C connection

const uint8_t I2C_SLAVE_ADDRESS = 0x10;

const uint I2C_SDA_PIN = 16;
const uint I2C_SCL = 17;

void connection_setup()
{
    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);

    gpio_init(I2C_SCL);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);

    i2c_init(i2c0, 100 * 1000);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

system_state_t get_system_state()
{
    return convert_to_system_state_t(context.memory);
}

drive_state_t get_drive_state()
{
    return convert_to_drive_state_t(context.memory);
}

flipper_state_t get_flipper_state()
{
    return convert_to_flipper_state_t(context.memory);
}
