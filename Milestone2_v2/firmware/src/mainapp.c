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
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t sendToMainAppQueue(messageStructure msg)
{
    return xQueueSend(mainappData.mainQueue, &msg, portMAX_DELAY);
}

BaseType_t sendToMainAppQueueFromISR(messageStructure msg)
{
    return xQueueSendFromISR(mainappData.mainQueue, &msg, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void sendMessage(void)
{
    messageStructure newMessage;
    // Deconstruct
    newMessage.sender = SERVER;
    newMessage.messageNumber = 0;
    newMessage.messageType = SENSOR;
    newMessage.messageSize = sizeof(unsigned int);
    newMessage.messageContent[0] = 0x00;
    newMessage.messageContent[1] = 0x00;
    newMessage.messageContent[2] = 0x01;
    newMessage.messageContent[3] = 0x0C;
    // Send
    sendToTXQueue(newMessage);
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
    mainappData.mainQueue = xQueueCreate(16, sizeof(messageStructure));
}


/******************************************************************************
  Function:
    void MAINAPP_Tasks ( void )

  Remarks:
    See prototype in mainapp.h.
 */

void MAINAPP_Tasks ( void )
{
messageStructure tempMsg;
int a = 0;
    
PLIB_USART_Enable(USART_ID_1);

sendMessage();

while(1) {
    if(xQueueReceive(mainappData.mainQueue, &tempMsg, portMAX_DELAY)) {
        // Filter messages
        if(tempMsg.messageType == ACKNOWLEDGE) {
            // Filter out content
            sendMessage();
        }
    }
}
}

 

/*******************************************************************************
 End of File
 */
