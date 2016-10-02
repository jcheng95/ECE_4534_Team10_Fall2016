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

#ifndef _MOTOR_CONTROL_PUBLIC_H    /* Guard against multiple inclusion */
#define _MOTOR_CONTROL_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "common.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

BaseType_t sendToMotorControlQueue(messageStructure msg);
BaseType_t sendToMotorControlQueueFromISR(messageStructure msg);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _MOTOR_CONTROL_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
