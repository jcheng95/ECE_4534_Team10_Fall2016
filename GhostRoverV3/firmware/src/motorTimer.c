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

#include "motorTimer.h"

void callbackMotorTimer(TimerHandle_t timer){
    motorCaller();
}

void initializeMotorTimer(){
    TimerHandle_t motorTimer;
    motorTimer = xTimerCreate
                 ( "motorTimerString",
                   (50 / portTICK_PERIOD_MS),
                   pdTRUE,
                   (void *) 2,
                   callbackMotorTimer);
    xTimerStart(motorTimer,0);
}