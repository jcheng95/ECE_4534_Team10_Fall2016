/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _SENSOR_TIMER_H    /* Guard against multiple inclusion */
#define _SENSOR_TIMER_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
    
#include <timers.h>
#include "motor_control_public.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

void sensorTimerInitialize();
void sensorTimerCallback(TimerHandle_t timing);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_TIMER_H */

/* *****************************************************************************
 End of File
 */
