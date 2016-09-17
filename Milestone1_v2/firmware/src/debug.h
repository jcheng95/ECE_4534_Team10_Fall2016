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
    // This will output debug values to pins 54-61
    // The pin order here is in reverse going from B0 - B7
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
    ////////////////////////////////////////////////////////////////////////
    void dbgOutputLoc(unsigned char outVal);
    //task consts
    
    #define TASK_ENTER ((unsigned char)'0')
    #define INF_WHILE ((unsigned char)'1')
    #define TASK_BEFORE_Q_TX ((unsigned char)'2')
    #define TASK_BEFORE_Q_RX ((unsigned char)'3')
    #define TASK_AFTER_Q_TX ((unsigned char)'4')
    #define TASK_AFTER_Q_RX ((unsigned char)'5')
    
    //ISR consts
    
    #define ENTER_ISR ((unsigned char)'6')
    #define EXIT_ISR ((unsigned char)'7')
    #define ISR_BEFORE_Q_TX ((unsigned char)'8')
    #define ISR_BEFORE_Q_RX ((unsigned char)'9')
    #define ISR_AFTER_Q_TX ((unsigned char)'a')
    #define ISR_AFTER_Q_RX ((unsigned char)'b')
    
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
