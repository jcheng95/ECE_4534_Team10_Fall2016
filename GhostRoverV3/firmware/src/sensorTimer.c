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

#include "sensorTimer.h"

void callbackSensorTimer(TimerHandle_t timer){
    irSensor();
}


void initializeSensorTimer(){
    TimerHandle_t irTimer;
    irTimer = xTimerCreate
                 ( "irTimerString",
                   (10 / portTICK_PERIOD_MS),
                   pdTRUE,
                   (void *) 2,
                   callbackSensorTimer);
    xTimerStart(irTimer,0);
}