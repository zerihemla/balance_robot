#include "rtosUtility.h"


void taskSleepMs(uint32_t msToSleep)
{
    vTaskDelay(pdMS_TO_TICKS(msToSleep));
}

uint32_t taskGetTimeMs(void)
{
    return TASK_GET_MS();
}