#include "mpu6050.h"

#include "boards/pico.h"
#include "hardware/i2c.h"
#include "logInterface.h"
#include "rtosUtility.h"
#include "pinout.h"


#include "hardware/gpio.h" //used to create the GPIO interrupt
#include "semphr.h"

#include "math.h"



typedef enum
{
    SELF_TEST_X_REG = 0x0D,    //1 byte
    SELF_TEST_Y_REG = 0x0E,    //1 byte
    SELF_TEST_Z_REG = 0x0F,    //1 byte
    
    GYRO_CONFIG_REG = 0x1B,    //1 byte
    ACCEL_CONFIG_REG = 0x1C,   //1 byte

    INT_PIN_CONFIG_REG = 0x37, //1 byte
    INT_ENABLE_REG = 0x38,     //1 byte

    ACCEL_REG = 0x3B,          //6 bytes
    TEMP_REG = 0x41,           //2 bytes
    GYRO_REG = 0x43,           //6 bytes

    WHO_AM_I_REG = 0x75,       //1 byte

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


#define MAX_PRINT_COUNT 15


#define INT16_T_MAX 32767

#define ACCEL_FULL_SCALE_VALUE (2.0f)
#define GYRO_FULL_SCALE_VALUE  (250.0f) 


#define ACCEL_SCALE_FACTOR (INT16_T_MAX / ACCEL_FULL_SCALE_VALUE)
#define GYRO_SCALE_FACTOR  (INT16_T_MAX / GYRO_FULL_SCALE_VALUE)

#define CHIP_SEM_READ_TIME 100

//The weights into the kalman filter
#define ACCEL_TRUST_FACTOR 0.05
#define GYRO_TRUST_FACTOR 1 - ACCEL_TRUST_FACTOR

/////////LOCAL VARS//////////////
static bool _mpuFound = 0;
uint8_t _whoAmIRead = 0;

uint8_t _printCount = 0;

static SemaphoreHandle_t _chipReadSem = NULL;
uint32_t _curSensorReadTimeMs = 0;
uint32_t _lastSensorReadTimeMs = 0;



/////////LOCAL FUNCTION PROTOTYPES/////////
//converting functions
static void _accelConvert(int16_t* inputAccel, float* outputAccel);
static void _gyroConvert(int16_t* inputGyro, float* outputGyro);
static void _tempConvert(int16_t* inputTemp, float* outputTemp);

//printing functions
static void _printRawValues(int16_t* rawAcceleration, int16_t* rawGyro, int16_t rawTemp);
static void _printConvertedValues(float* unitAcceleration, float* unitGyro, float unitTemp);
static void _printAngles(float* accelAngleOutput, float* gyroAngleOutput, float* filteredAngleOutput);

//calculate angle functions
static void _calculateAccelAngles(float* unitAccelInput, float* accelAngleOutput);
static void _calculateGyroAngles(float* unitGyro, float* lastAngles, float* gyroAngleOutput);
static void _combineAngles(float* accelAngles, float* gyroAngles, float* filteredAngles);

//interrupt fucntions
static void _interruptCallback(uint gpio, uint32_t events);
void _getEventString(char *buf, uint32_t events);



/////////////////////////////////////////////////////////
////////////PUBLIC FUNCTIONS/////////////////////////////
/////////////////////////////////////////////////////////
void mpu6050_init(void)
{
    _mpuFound = mpu6050_find();

    mpu6050_reset();

    //if these set config values are changed, you also 
    //need to change the FULL_SCALE_VALUE #defines.

    //Set the flull scale range to +/- 250dps
    uint8_t setGyroConfig = 0x00;

    //Set the full scale range to +- 2g
    uint8_t setAccelConfig = 0x00;

    //Sets the interrupt to fire when there is data ready.
    uint8_t interruptEnableConfig = 0x01;

    //sets the interrupt to be high, and stay till a read happens.
    uint8_t interruptConfig = 0x30;

    //set the int pin as a input pint
    gpio_set_dir(MPU_INT_PIN, 0);

    i2c_regWrite(MPU_ADDR, GYRO_CONFIG_REG, &setGyroConfig, 1);
    i2c_regWrite(MPU_ADDR, ACCEL_CONFIG_REG, &setAccelConfig, 1);

    i2c_regWrite(MPU_ADDR, INT_ENABLE_REG, &interruptEnableConfig, 1);
    i2c_regWrite(MPU_ADDR, INT_PIN_CONFIG_REG, &interruptConfig, 1);

    if (_mpuFound)
    {
        gpio_set_irq_enabled_with_callback(MPU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, &_interruptCallback);
    }

    _chipReadSem = xSemaphoreCreateBinary();
}


void mpu6050_reset() 
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDR, buf, 2, false);
}

bool mpu6050_find(void)
{
    uint8_t buffer [1];
    int numRead = i2c_regRead(MPU_ADDR, WHO_AM_I_REG, buffer, 1);

    _whoAmIRead = buffer[0];

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
    _lastSensorReadTimeMs = _curSensorReadTimeMs;
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    i2c_regRead(MPU_ADDR, ACCEL_REG, buffer, 6);
    //i2c_write_blocking(i2c_default, MPU_ADDR, &val, 1, true); // true to keep master control of bus
    //i2c_read_blocking(i2c_default, MPU_ADDR, buffer, 6, false);


    uint8_t readIndex = 0;
    for (int i = 0; i < 3; i++) 
    {
        accel[i] = (buffer[readIndex] << 8 | buffer[readIndex + 1]);
        readIndex += 2;
    }

    // Now gyro data from reg 0x43 for 6 bytes
    i2c_regRead(MPU_ADDR, GYRO_REG, buffer, 6);
    // i2c_write_blocking(i2c_default, MPU_ADDR, &val, 1, true);
    // i2c_read_blocking(i2c_default, MPU_ADDR, buffer, 6, false);  // False - finished with bus

    readIndex = 0;
    for (int i = 0; i < 3; i++) 
    {
        gyro[i] = (buffer[readIndex] << 8 | buffer[readIndex + 1]);
        readIndex += 2;
    }

    // Now temperature from reg 0x41 for 2 bytes
    uint8_t tempReg = 0x41;
    i2c_regRead(MPU_ADDR, tempReg, buffer, 2);
    // i2c_write_blocking(i2c_default, MPU_ADDR, &val, 1, true);
    // i2c_read_blocking(i2c_default, MPU_ADDR, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];

    _curSensorReadTimeMs = taskGetTimeMs();
}



void mpu6050_task() 
{
    int16_t rawAcceleration[3], rawGyro[3], rawTemp;

    float unitAcceleration[3], unitGyro[3], unitTemp;

    float accelAngles[2], gyroAngles[2], filteredAngles[2], lastAngles[2];


    while (1) 
    {
        if(!_mpuFound)
        {
            log_printOutput("MPU6050 was not found in init!");
            printf("%d Read Who Am I : %d", _whoAmIRead);
            i2c_busScan();
            taskSleepMs(1000);
            continue;
        }

        //Wait till a measurment is signlaed by hte interrupt.
        xSemaphoreTake(_chipReadSem, CHIP_SEM_READ_TIME);



        //Read from the sensor
        mpu6050_read_raw(rawAcceleration, rawGyro, &rawTemp);
        


        //Convert Mesaurements to something useful
        _accelConvert(rawAcceleration, unitAcceleration);
        _gyroConvert(rawGyro, unitGyro);
        _tempConvert(&rawTemp, &unitTemp);



        //Calculate the angles
        lastAngles[0] = filteredAngles[0];
        lastAngles[1] = filteredAngles[1];

        _calculateAccelAngles(unitAcceleration, accelAngles);
        _calculateGyroAngles(unitGyro, lastAngles, gyroAngles);

        _combineAngles(accelAngles, gyroAngles, filteredAngles);


        //Print out the angles.
        _printCount++;
        if(_printCount >= MAX_PRINT_COUNT)
        {
            _printCount = 0;

            printf("*****************************\n");
            _printRawValues(rawAcceleration, rawGyro, rawTemp);
            _printConvertedValues(unitAcceleration, unitGyro, unitTemp);
            
            _printAngles(accelAngles, gyroAngles, filteredAngles);
        }

        //Waits are bad. DOnt wait.
        // taskSleepMs(100);
    }
}



/////////////////////////////////////////////////////
/////////LOCAL FUNCTIONS/////////////////////////////
/////////////////////////////////////////////////////

////CONVERT FUNCTIONS////

//changes the units from raw to deg/sec
static void _accelConvert(int16_t* inputAccel, float* outputAccel)
{
    for (int i = 0; i < 3; i ++)
    {
        outputAccel[i] = (float)inputAccel[i] / ACCEL_SCALE_FACTOR;
    }
    return;
}

//changes the units from raw to G's
static void _gyroConvert(int16_t* inputGyro, float* outputGyro)
{
    for (int i = 0; i < 3; i ++)
    {
        outputGyro[i] = (float)inputGyro[i] / GYRO_SCALE_FACTOR;
    }
    return;
}

//changes the units from raw to deg C
static void _tempConvert(int16_t* inputTemp, float* outputTemp)
{
    *outputTemp = (*inputTemp / 340.0) + 36.53;
}


////PRINT FUNCTIONS/////

static void _printRawValues(int16_t* rawAcceleration, int16_t* rawGyro, int16_t rawTemp)
{
    // These are the raw numbers from the chip, so will need tweaking to be really useful.
    // See the datasheet for more information
    printf("RAW Acc. X = %d, Y = %d, Z = %d\n", rawAcceleration[0], rawAcceleration[1], rawAcceleration[2]);
    printf("RAW Gyro. X = %d, Y = %d, Z = %d\n", rawGyro[0], rawGyro[1], rawGyro[2]);
    // Temperature is simple so use the datasheet calculation to get deg C.
    // Note this is chip temperature.
    printf("RAW Temp. = %f\n\n", rawTemp);
}

static void _printConvertedValues(float* unitAcceleration, float* unitGyro, float unitTemp)
{
    printf("unit Acc. X = %.2f g, Y = %.2f g, Z = %.2f g\n", unitAcceleration[0], unitAcceleration[1], unitAcceleration[2]);
    printf("unit Gyro. X = %.2f m/s2, Y = %.2f m/s2, Z = %.2f m/s2\n", unitGyro[0], unitGyro[1], unitGyro[2]);
    printf("unit Temp. = %.2f C \n\n", unitTemp);
}

static void _printAngles(float* accelAngleOutput, float* gyroAngleOutput, float* filteredAngleOutput)
{
    printf("Accel: Angle X = %.2f, Y = %.2f\n", accelAngleOutput[0], accelAngleOutput[1]);
    printf("Gyro:  Angle X = %.2f, Y = %.2f\n", gyroAngleOutput[0], gyroAngleOutput[1]);
    printf("Filt:  Angle X = %.2f, Y = %.2f\n", filteredAngleOutput[0], filteredAngleOutput[1]);
}


////CALCULATE ANGLE FUNCTIONS////

static void _calculateAccelAngles(float* unitAccelInput, float* accelAngleOutput)
{

    float x = unitAccelInput[0];
    float y = unitAccelInput[1];
    float z = unitAccelInput[2];

    float x2 = x*x;
    float y2 = y*y;
    float z2 = z*z;

    accelAngleOutput[0] = atan(x/(sqrt(y2 + z2)));
    accelAngleOutput[1] = atan(y/(sqrt(x2 + z2)));

    accelAngleOutput[0] = accelAngleOutput[0] * 180 / M_PI;
    accelAngleOutput[1] = accelAngleOutput[1] * 180 / M_PI;
}

static void _calculateGyroAngles(float* unitGyro, float* lastAngles, float* gyroAngleOutput)
{
    if(_lastSensorReadTimeMs == 0)
    {
        //This is the first time through the loop. We cant calculate gyro angles.
        return;
    }
    
    uint32_t timeDeltaMs = _curSensorReadTimeMs - _lastSensorReadTimeMs;
    float timeDeltaSec = ((float) timeDeltaMs / 1000);


    //I dont know if this is the right gyro axis here, it should be re-examined.
    gyroAngleOutput[0] = lastAngles[0] + (unitGyro[0] * timeDeltaSec);
    gyroAngleOutput[1] = lastAngles[1] + (unitGyro[1] * timeDeltaSec);
}


static void _combineAngles(float* accelAngles, float* gyroAngles, float* filteredAngles)
{
    if (_lastSensorReadTimeMs == 0)
    {
        //This is the first time reading the sensor. The gyro wont have calculated angles yet.
        filteredAngles[0] = accelAngles[0];
        filteredAngles[1] = accelAngles[1];
        return;
    }


    for (int i = 0; i < 2; i ++)
    {
        filteredAngles[i] = (ACCEL_TRUST_FACTOR * accelAngles[i]) + (GYRO_TRUST_FACTOR * gyroAngles[i]);
    }
}


////INTERRUPT HANDLER CODE////

static void _interruptCallback(uint gpio, uint32_t events) 
{
    //The next three lines are only for debugging.
    
    // static char event_str[128];
    // _getEventString(event_str, events);
    // printf("GPIO %d %s\n", gpio, event_str);

    //Give the semaphore from ISR
    //I probably need to make a semaphore first....
    
    BaseType_t taskWoke = 0; //I dont know what this does...
    xSemaphoreGiveFromISR(_chipReadSem, &taskWoke);
}

static const char *gpio_irq_str[] = 
{
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void _getEventString(char *buf, uint32_t events) 
{
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

