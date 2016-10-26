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


#include "softwareTimer.h"



void callbackTimer(TimerHandle_t timer){
    irSensor();
}


void initializeTimer(){
    TimerHandle_t irTimer;
    irTimer = xTimerCreate
                 ( "irTimerString",
                   (10 / portTICK_PERIOD_MS),
                   pdTRUE,
                   (void *) 2,
                   callbackTimer);
    xTimerStart(irTimer,0);
}



/* *****************************************************************************
 End of File
 */
