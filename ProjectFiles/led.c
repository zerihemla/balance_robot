#include "led.h"
#include "rtosUtility.h"
#include "logInterface.h"

#define LED_TASK_PERIOD_MS 100

#define ONBOARD_LED_PIN PICO_DEFAULT_LED_PIN

                                   //INIT, IDLE, ACTIVE, PANIC
uint16_t onTimes[LED_NUM_STATES]  = {700, 100,  500,    50};
uint16_t offTimes[LED_NUM_STATES] = {100,  700,  500,    50};


//////Local Vars////////////////////////
onboardLedState_t _curOnboardLedState;

SemaphoreHandle_t _ledMutex;

uint32_t _lastTime = 0;
uint32_t _curTime = 0;
bool _onboardLedOn = false;

uint32_t _taskSwitchTime = 0;

//////Local Function Prototypes/////////
void _setOnboardLed(bool ledState);
void _onboardLedStateHandler(void);


void led_init(void)
{
    gpio_init(ONBOARD_LED_PIN);
    gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

    _setOnboardLed(1);
    _lastTime = taskGetTimeMs();
    _taskSwitchTime = taskGetTimeMs();

    _ledMutex = xSemaphoreCreateBinary();

    onboardLedState_t _curOnboardLedState = LED_STATE_INIT;
}


void led_task()
{   

    while (true) 
    {
        _curTime = taskGetTimeMs();
        _onboardLedStateHandler();

        taskSleepMs(LED_TASK_PERIOD_MS);
    }
}

void led_setOnboardLedState(onboardLedState_t newLedState)
{
    MUTEX_LOCK(_ledMutex);
    log_printOutput("New Led State!");
    _curOnboardLedState = newLedState;
    MUTEX_UNLOCK(_ledMutex);
}

onboardLedState_t led_getOnboardLedState(void)
{
    onboardLedState_t retVal = 0;
    MUTEX_LOCK(_ledMutex);
    retVal = _curOnboardLedState;
    MUTEX_UNLOCK(_ledMutex);
    return retVal;
}




////////////////////////////////////////////
///////////LOCAL FUNCTIONS//////////////////
////////////////////////////////////////////

void _setOnboardLed(bool ledState)
{
    _onboardLedOn = ledState;
    gpio_put(ONBOARD_LED_PIN, ledState);
}


void _onboardLedStateHandler(void)
{
    onboardLedState_t curLedState = led_getOnboardLedState();

    if(_onboardLedOn)
    {
        uint16_t onTime = onTimes[curLedState];

        if ((_curTime - _lastTime) > onTime)
        {
            _lastTime = _curTime;
            _setOnboardLed(0);
        }
    }

    else
    {
        uint16_t offTime = offTimes[curLedState];

        if ((_curTime - _lastTime) > offTime)
        {
            _lastTime = _curTime;
            _setOnboardLed(1);
        }
    }

}
