#include "i2cInterface.h"

#include "logInterface.h"


i2c_inst_t* _i2c;

const uint sda_pin = 6;
const uint scl_pin = 7;


//////Local Function Prototypes///////
bool _isReservedAddr(uint8_t addr);


//////////////////////////////////////////////
////////////PUBLIC FUNCTIONS//////////////////
//////////////////////////////////////////////

void i2c_interfaceInit(void)
{
    // // Ports
    // _i2c = i2c0;

    // //Initialize I2C port at 400 kHz
    // //speed is changed by changing the second arg
    // i2c_init(_i2c, 400 * 1000);

    // // Initialize I2C pins
    // gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    // gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    // gpio_pull_up(sda_pin);
    // gpio_pull_up(scl_pin);

    // // Make the I2C pins available to picotool
    // bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));


    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
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
    i2c_write_blocking(i2c_default, addr, msg, (nbytes + 1), false);

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
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c_default, addr, buf, nbytes, false);

    return num_bytes_read;
}

















// Sweep through all 7-bit I2C addresses, to see if any slaves are present on
// the I2C bus. Print out a table that looks like this:
//
// I2C Bus Scan
//   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
// 0
// 1       @
// 2
// 3             @
// 4
// 5
// 6
// 7
//
// E.g. if slave addresses 0x12 and 0x34 were acknowledged.

void i2c_busScan() 
{
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr)
     {
        if (addr % 16 == 0) 
        {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (_isReservedAddr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("i2c scan done.\n");
}



void i2c_testTask()
{
    while(1)
    {
        i2c_busScan();
    }
}

//////////////////////////////////////////////////
/////LOCAL FUNCTIONS//////////////////////////////
//////////////////////////////////////////////////


// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool _isReservedAddr(uint8_t addr) 
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
