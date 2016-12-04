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

#include "adcTimer.h"

void callbackADCTimer(TimerHandle_t timer){
    incrementCounter();
}

void initializeADCTimer(){
    TimerHandle_t adc_Timer;
    adc_Timer = xTimerCreate
                 ( "adcTimerString",
                   (500 / portTICK_PERIOD_MS),
                   pdTRUE,
                   (void *) 2,
                   callbackADCTimer);
    xTimerStart(adc_Timer,0);
}