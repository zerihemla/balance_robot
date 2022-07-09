#include "mpu6050.h"

#include "boards/pico.h"
#include "hardware/i2c.h"
#include "logInterface.h"
#include "rtosUtility.h"


typedef enum
{
    selfTestX = 0x0D,   //1 byte
    selfTestY = 0x0E,   //1 byte
    selfTestZ = 0x0F,   //1 byte
    
    gyroConfig = 0x1B,  //1 byte
    accelConfig = 0x1C, //1 byte

    intPinConfig = 0x37,//1 byte
    intEnable = 0x38,   //1 byte

    accelReg = 0x38,    //6 bytes
    tempReg = 0x41,     //2 bytes
    gyroReg = 0x43,     //6 bytes
    whoAmI = 0x75,      //1 byte

}mpu6050Reg_t;




    /**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050 board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/

// By default these devices  are on bus MPU_ADDRess 0x68
#define MPU_ADDR 0x68 
#define MPU_WHO_AM_I_VALUE 0x68

static bool _mpuFound = 0;

void mpu6050_init(void)
{
    _mpuFound = mpu6050_find();

    mpu6050_reset();

    // #endif

}


#ifdef i2c_default
void mpu6050_reset() 
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDR, buf, 2, false);
}

bool mup6050_find(void)
{
    uint8_t buffer [1];
    int numRead = i2c_regRead(MPU_ADDR, whoAmI, buffer, 1);

    if (numRead == 1)
    {
        if (buffer[0] == MPU_WHO_AM_I_VALUE)
        {
            log_printOutput("MPU6050 Found!");
            return true;
        }
    }

    log_printOutput("MPU6050 NOT Found!");
    return false; 
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) 
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    i2c_regRead(MPU_ADDR, accelReg, buffer, 6);
    //i2c_write_blocking(i2c_default, MPU_ADDR, &val, 1, true); // true to keep master control of bus
    //i2c_read_blocking(i2c_default, MPU_ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) 
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    i2c_regRead(MPU_ADDR, gyroReg, buffer, 6);
    // i2c_write_blocking(i2c_default, MPU_ADDR, &val, 1, true);
    // i2c_read_blocking(i2c_default, MPU_ADDR, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) 
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    uint8_t tempReg = 0x41;
    i2c_regRead(MPU_ADDR, tempReg, buffer, 2);
    // i2c_write_blocking(i2c_default, MPU_ADDR, &val, 1, true);
    // i2c_read_blocking(i2c_default, MPU_ADDR, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
#endif



int mpu6050_task() 
{
    int16_t acceleration[3], gyro[3], temp;

    while (1) 
    {
        if(!_mpuFound)
        {
            log_printOutput("MPU6050 was not found in init!");
            taskSleepMs(1000);
        }

        mpu6050_read_raw(acceleration, gyro, &temp);

        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        taskSleepMs(100);
    }


    return 0;
}

