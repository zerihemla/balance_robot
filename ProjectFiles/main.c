#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "i2cInterface.h"

#include "led.h"
#include "logInterface.h"
#include "tester.h"
#include "mpu6050.h"



// #define I2C_SCAN




void startTasks(void)
{
    //Normal Operation
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(testTask, "Tester_Task", 256, NULL, 1, NULL);
    
    #ifndef I2C_SCAN
    xTaskCreate(mpu6050_task, "MPU6050_Task", 256, NULL, 1, NULL);
    
    //I2C Scan
    #else
    xTaskCreate(i2c_testTask, "I2C_Test", 256, NULL, 1, NULL);


    #endif

    vTaskStartScheduler();
}


void startupDelay(void)
{
    log_printOutput("Starting Init in...");

    for (int i = 10; i >= 0; i --)
    {
        printf("%d", i);
        taskSleepMs(1000);
    }
}




int main()
{
    //Init all GPIO
    stdio_init_all();
    //Init Functions
    // startupDelay();

    //Init the LED
    led_init();

    //Init the i2c
    i2c_interfaceInit();

    //init the Gyro/Accel
    mpu6050_init();

    //Start Tasks
    startTasks();

    while(1)
    {
        //dont do anything here...
    };
}
