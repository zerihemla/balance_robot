#include "tester.h"
#include "led.h"

#include "logInterface.h"


void testTask(void)
{
    while(1)
    {
        log_printOutput("Started Tester Loop");
        for (int i = 0; i < LED_NUM_STATES; i ++)
        {
            log_printOutput("Tester Checkin");
            led_setOnboardLedState(i);
            taskSleepMs(5000);
        }
    }    
}