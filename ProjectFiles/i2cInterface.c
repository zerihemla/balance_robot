#include "i2cInterface.h"


i2c_inst_t* _i2c;

const uint sda_pin = 4;
const uint scl_pin = 5;


void i2c_interfaceInit(void)
{
    // Ports
    _i2c = i2c0;

    //Initialize I2C port at 400 kHz
    //speed is changed by changing the second arg
    i2c_init(_i2c, 400 * 1000);

    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));
}


int i2c_regWrite(const uint addr, const uint8_t reg, 
                 uint8_t *buf, const uint8_t nbytes) 
{

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) 
    {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) 
    {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(_i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int i2c_regRead(const uint addr, const uint8_t reg,
                uint8_t *buf, const uint8_t nbytes) 
{
    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) 
    {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(_i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(_i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}