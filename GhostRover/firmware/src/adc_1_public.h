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

#ifndef _ADC_1_PUBLIC_H    /* Guard against multiple inclusion */
#define _ADC_1_PUBLIC_H


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

BaseType_t sendToADCAppQueue(unsigned int msg);
BaseType_t sendToADCAppQueueFromISR(unsigned int msg);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _ADC_1_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
