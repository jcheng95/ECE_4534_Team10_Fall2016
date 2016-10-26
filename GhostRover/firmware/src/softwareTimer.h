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

#ifndef _EXAMPLE_FILE_NAME_H    /* Guard against multiple inclusion */
#define _EXAMPLE_FILE_NAME_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */

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

//timer functions
    void initializeTimer();
    void callbackTimer(TimerHandle_t timer);
    
    
/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
