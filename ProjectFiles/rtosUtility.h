#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"

#define MUTEX_LOCK(mutex) if (mutex != NULL) { if (xSemaphoreTake(mutex, (TickType_t) portMAX_DELAY) == pdTRUE) {
#define MUTEX_UNLOCK(mutex) xSemaphoreGive(mutex); }}

#define TASK_GET_MS() (xTaskGetTickCount());

void taskSleepMs(uint32_t msToSleep);
uint32_t taskGetTimeMs(void);