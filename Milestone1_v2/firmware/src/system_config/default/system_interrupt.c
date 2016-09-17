/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "mainapp.h"
#include "system_definitions.h"

#include "debug.h"
#include "mainapp_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

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

void IntHandlerDrvTmrInstance0(void)
{
    // Immediately upon entering an ISR
    dbgOutputLoc(ENTER_ISR);
    
    // Clear the interrupt flag
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
    
    // Immediately before sending to a queue or receiving from a queue
    dbgOutputLoc(ISR_BEFORE_Q_TX);
    
    //Send to the message queue belonging to mainapp from the ISR
    mainAppSendTimerValToMsgQFromISR(50);
    
    // Immediately after sending to a queue or receiving from a queue
    dbgOutputLoc(ISR_AFTER_Q_TX);
    
    // Immediately before leaving an ISR
    dbgOutputLoc(EXIT_ISR);
}
  
/*******************************************************************************
 End of File
*/

