#ifndef _MAINAPP_PUBLIC_H    /* Guard against multiple inclusion */
#define _MAINAPP_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    ///////////////////////////////////////////////////////////////////////////////////
    //
    //  Function Name:  mainAppSendTimerValToMsgQ(unsigned int millisecondsElapsed)
    //
    //  Returns:        int
    //
    //  Parameters:     unsigned int millisecondsElapsed
    //
    //  Description:    Sends to the mainapp queue normally with a delay value of
    //                  portMAX_DELAY.
    //
    ///////////////////////////////////////////////////////////////////////////////////
    int mainAppSendTimerValToMsgQ(unsigned int millisecondsElapsed);
    
    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Function Name:  mainAppSendTimerValToMsgQFromISR(unsigned int millisecondsElapsed)
    //
    //  Returns:        int
    //
    //  Parameters:     unsigned int millisecondsElapsed
    //
    //  Description:    Sends to the mainapp queue from the ISR with a delay value of 0 so
    //                  that the ISR is not delayed and the other processes can be returned
    //                  to as quickly as possible.
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    int mainAppSendTimerValToMsgQFromISR(unsigned int millisecondsElapsed);
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
