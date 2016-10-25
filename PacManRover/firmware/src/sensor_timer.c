/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "sensor_timer.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

void sensorTimerInitialize()
{
    TimerHandle_t sensorTiming;
    //  Timer should be having a period of 10ms or 0.01s
    sensorTiming = xTimerCreate("Sensor", (10 / portTICK_PERIOD_MS), pdTRUE, (void *)2, sensorTimerCallback);
    xTimerStart(sensorTiming, 0);
}

void sensorTimerCallback(TimerHandle_t timing)
{
    // Sift through the state machine
    updateSensorData();
}

/* *****************************************************************************
 End of File
 */
