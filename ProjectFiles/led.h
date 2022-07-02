#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "rtosUtility.h"

typedef enum
{
    LED_STATE_INIT,
    LED_STATE_IDLE,
    LED_STATE_ACTIVE,
    LED_STATE_PANIC,
    LED_NUM_STATES,
}onboardLedState_t; 


void led_init(void);

void led_task();

void led_setOnboardLedState(onboardLedState_t newLedState);

onboardLedState_t led_getOnboardLedState(void);