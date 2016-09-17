/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    mainapp.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "mainapp.h"
#include "mainapp_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

MAINAPP_DATA mainappData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

unsigned int checkCounter(void)
{
    // Checks if the message has been sent 5 times
    if(mainappData.counter == 35) {
        return ERROR_CODE;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t mainAppSendTimerValToMsgQ(unsigned int millisecondsElapsed)
{
    return xQueueSend(mainappData.mainAppQueue, &millisecondsElapsed, portMAX_DELAY);
}

BaseType_t mainAppSendTimerValToMsgQFromISR(unsigned int millisecondsElapsed)
{
    return xQueueSendFromISR(mainappData.mainAppQueue, &millisecondsElapsed, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MAINAPP_Initialize ( void )

  Remarks:
    See prototype in mainapp.h.
 */

void MAINAPP_Initialize ( void )
{
    // Initialize queue
    // Queue is now 8 messages of 4 bytes length
    mainappData.mainAppQueue = xQueueCreate(8, sizeof(unsigned int));
    
    // Initialize state machine
    // The letter that will be sent is current the character 'T'
    mainappData.states = stateT;
    
    // Activates the timer driver
    // Required in order for the timer to work
    DRV_TMR0_Start();
    
    // Initialize the counter
    mainappData.counter = 0;
}


/******************************************************************************
  Function:
    void MAINAPP_Tasks ( void )

  Remarks:
    See prototype in mainapp.h.
 */

/*
 //////////////////////////////////////////
 Name                       ASCII Value
 ________________________________________
TASK_ENTER              0
INF_WHILE               1
TASK_BEFORE_Q_TX        2
TASK_BEFORE_Q_RX        3
TASK_AFTER_Q_TX         4
TASK_AFTER_Q_RX         5
ENTER_ISR               6
EXIT_ISR                7
ISR_BEFORE_Q_TX         8
ISR_BEFORE_Q_RX         9
ISR_AFTER_Q_TX          a
ISR_AFTER_Q_RX          b
-----------------------------------------
ERROR_CODE          0xFFFFFFFF
 
//////////////////////////////////////////
 */    

void MAINAPP_Tasks ( void )
{
    // Immediately upon entering the task
    dbgOutputLoc(TASK_ENTER);
    
    // Temporary value to get the new message in the queue
    unsigned int msg;
    
    // Immediately before the while(1)
    dbgOutputLoc(INF_WHILE);
    while(1) {
        // Immediately before sending to a queue or receiving from a queue
        dbgOutputLoc(TASK_BEFORE_Q_RX);
        // Checks if a message is available (blocking)
        if(xQueueReceive(mainappData.mainAppQueue, &msg, portMAX_DELAY)) {
            // Stopping at a certain value
            stopProcess(checkCounter());
            
            // Handle new message received
            switch(mainappData.states) {
                case stateT:
                    dbgOutputVal('T');
                    mainappData.states = stateE;
                    break;
                case stateE:
                    dbgOutputVal('E');
                    mainappData.states = stateA;
                    break;
                case stateA:
                    dbgOutputVal('A');
                    mainappData.states = stateM;
                    break;
                case stateM:
                    dbgOutputVal('M');
                    mainappData.states = state1;
                    break;
                case state1:
                    dbgOutputVal('1');
                    mainappData.states = state0;
                    break;
                case state0:
                    dbgOutputVal('0');
                    mainappData.states = stateT;
                    break;
                default:
                    mainappData.states = stateT;
                    break;
            }
            // Immediately after sending to a queue or receiving from a queue
            dbgOutputLoc(TASK_AFTER_Q_RX);
            // Increment the counter for the stopProcess() function
            mainappData.counter++;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
