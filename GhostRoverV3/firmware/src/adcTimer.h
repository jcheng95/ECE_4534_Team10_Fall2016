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

#ifndef _ADCTIMER_H    /* Guard against multiple inclusion */
#define _ADCTIMER_H


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
#include "adc_app_public.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

void callbackADCTimer(TimerHandle_t timer);
void initializeADCTimer();

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _ADCTIMER_H */

/* *****************************************************************************
 End of File
 */
