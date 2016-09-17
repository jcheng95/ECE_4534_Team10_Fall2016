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

#ifndef _TX_BUFFER_PUBLIC_H    /* Guard against multiple inclusion */
#define _TX_BUFFER_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

void initializeBufferQueue();
BaseType_t sendToTXBufferQueue(char msg);
BaseType_t sendToTXBufferQueueFromISR(char msg);
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _TX_BUFFER_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
