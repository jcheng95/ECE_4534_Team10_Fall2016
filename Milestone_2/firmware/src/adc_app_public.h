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

#ifndef _ADC_APP_PUBLIC_H    /* Guard against multiple inclusion */
#define _ADC_APP_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    BaseType_t sendToADCQueue(unsigned short val);
    BaseType_t sendToADCQueueFromISR(unsigned short val);
    void activateReadySignal(void);
    void addToSampleValue(ADC_SAMPLE val);
    void averageSample(void);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _ADC_APP_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
