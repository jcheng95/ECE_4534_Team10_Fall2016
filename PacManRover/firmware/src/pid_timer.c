#include "pid_timer.h"

void timerCallback(TimerHandle_t timing)
{
    // Make call to the PID algorithm for correction
    pidAdjustment();
}

void timerInitialize()
{
    TimerHandle_t timing;
    //  Timer should be having a period of 50ms or 0.05s
    timing = xTimerCreate("Timer", (50 / portTICK_PERIOD_MS), pdTRUE, (void *)2, timerCallback);
    xTimerStart(timing, 0);
}