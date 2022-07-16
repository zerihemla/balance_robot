#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "i2cInterface.h"

#include "pico.h"

// #define PICO_DEFAULT_I2C         // Define the default I2C for a board, min=0, max=1, group=hardware_i2c
// #define PICO_DEFAULT_I2C_SDA_PIN // Define the default I2C SDA pin, min=0, max=29, group=hardware_i2c
// #define PICO_DEFAULT_I2C_SCL_PIN // Define the default I2C SCL pin, min=0, max=29, group=hardware_i2c


void mpu6050_init(void);

void mpu6050_reset(void);

bool mpu6050_find(void);

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

void mpu6050_task();