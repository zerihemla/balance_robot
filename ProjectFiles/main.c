#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"

#include "led.h"
#include "logInterface.h"
#include "tester.h"




void startTasks(void)
{
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(testTask, "Tester_Task", 256, NULL, 1, NULL);


    vTaskStartScheduler();
}




int main()
{
    //Init Functions
    stdio_init_all();
    led_init();

    //Start Tasks
    startTasks();

    while(1)
    {
    };
}
