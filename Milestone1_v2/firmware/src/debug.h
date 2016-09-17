#ifndef _DEBUG_H    /* Guard against multiple inclusion */
#define _DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
    // Task constants
    #define TASK_ENTER ((unsigned char)'0')
    #define INF_WHILE ((unsigned char)'1')
    #define TASK_BEFORE_Q_TX ((unsigned char)'2')
    #define TASK_BEFORE_Q_RX ((unsigned char)'3')
    #define TASK_AFTER_Q_TX ((unsigned char)'4')
    #define TASK_AFTER_Q_RX ((unsigned char)'5')
    
    //ISR constants
    #define ENTER_ISR ((unsigned char)'6')
    #define EXIT_ISR ((unsigned char)'7')
    #define ISR_BEFORE_Q_TX ((unsigned char)'8')
    #define ISR_BEFORE_Q_RX ((unsigned char)'9')
    #define ISR_AFTER_Q_TX ((unsigned char)'a')
    #define ISR_AFTER_Q_RX ((unsigned char)'b')

    // Error code
    #define ERROR_CODE ((unsigned int)0xFFFFFFFF)
    #define ERROR_STATE ((unsigned char) 'Q')
    
    ////////////////////////////////////////////////////////////////////////
    //
    //  Function Name: dbgOutputVal(unsigned char outVal)
    //
    //  Returns:        void
    //
    //  Parameters:     unsigned char outVal
    //
    //  Description:    This function outputs the value "Team 10" byte by
    //                  byte to the GPIO pins 30 - 37. These GPIO pins
    //                  represent the pin order E7 - E0, respectively.
    //
    ////////////////////////////////////////////////////////////////////////
    void dbgOutputVal(unsigned char outVal);
    
    ////////////////////////////////////////////////////////////////////////
    // This will output debug values to pins 83, 13, 82, 12, 81, 11, 80, 10
    // The pin order here is:
    // RG12, RA3, RG14, RA2, RA7, RC4, RA6, RD4
    //
    //  Function Name: dbgOutputVal(unsigned char outVal)
    //
    //  Returns:        void
    //
    //  Parameters:     unsigned char outVal
    //
    //  Description:    This function outputs the value of our states byte
    //                  by byte to the GPIO pins 54 - 61. These GPIO pins
    //                  represent the pin order B0 - B7, respectively.
    //
    //  NOTE:           It is strongly recommended to not use this outside of
    //                  milestone 1. The pins used here will be in the way of
    //                  other pins because they are not directly (logically)
    //                  related.
    //
    ////////////////////////////////////////////////////////////////////////
    void dbgOutputLoc(unsigned char outVal);
    
    ////////////////////////////////////////////////////////////////////////
    //
    //  Function Name: stopProcess(unsigned int outVal)
    //
    //  Returns:        void
    //
    //  Parameters:     unsigned int outVal
    //
    //  Description:    This function stops the process if a value put
    //                  in the parameter is incorrect.
    //
    ////////////////////////////////////////////////////////////////////////
    void stopProcess(unsigned int outVal);
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
