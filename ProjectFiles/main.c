#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "i2cInterface.h"

#include "led.h"
#include "logInterface.h"
#include "tester.h"




void startTasks(void)
{
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(testTask, "Tester_Task", 256, NULL, 1, NULL);


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

    //Init Functions
    // startupDelay();

    //Init the LED
    led_init();

    //Init the i2c
    // i2c_interfaceInit();

    stdio_init_all();


    //Start Tasks
    startTasks();

    while(1)
    {
        //dont do anything here...
    };
}
