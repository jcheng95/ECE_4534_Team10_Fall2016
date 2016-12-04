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

#ifndef _UART_RX_PUBLIC_H    /* Guard against multiple inclusion */
#define _UART_RX_PUBLIC_H


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

BaseType_t sendToRXQueue(messageStructure msg);
BaseType_t sendToRXQueueFromISR(messageStructure msg);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _UART_RX_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
