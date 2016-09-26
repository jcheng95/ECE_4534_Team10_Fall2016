/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uart_tx.c

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

#include "uart_tx.h"

#include "uart_tx_public.h"

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

UART_TX_DATA uart_txData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t sendToTXQueue(messageStructure msg)
{
    xQueueSend(uart_txData.txQueue, &msg, portMAX_DELAY);
}

BaseType_t sendToTXQueueFromISR(messageStructure msg)
{
    xQueueSendFromISR(uart_txData.txQueue, &msg, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void sendMessageToBuffer(messageStructure msg)
{
    // Send message
    int i;
    sendToTXBufferQueue(START_BYTE);
    sendToTXBufferQueue(PIC);   // Sender
    sendToTXBufferQueue(uart_txData.messageCounter);
    sendToTXBufferQueue(msg.messageType);
    sendToTXBufferQueue(msg.messageSize);
    for(i = 0; i < msg.messageSize; i++) {
        sendToTXBufferQueue(msg.messageContent[i]);
    }
    sendToTXBufferQueue(END_BYTE);
    
    // Increment counter and account for overflow
    uart_txData.messageCounter++;
    uart_txData.messageCounter %= 256;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UART_TX_Initialize ( void )

  Remarks:
    See prototype in uart_tx.h.
 */

void UART_TX_Initialize ( void )
{
    uart_txData.txQueue = xQueueCreate(16, sizeof(messageStructure));
    uart_txData.messageCounter = 0;
}


/******************************************************************************
  Function:
    void UART_TX_Tasks ( void )

  Remarks:
    See prototype in uart_tx.h.
 */

void UART_TX_Tasks ( void )
{
    messageStructure sendingMessage;
    
    while(1) {
        if(xQueueReceive(uart_txData.txQueue, &sendingMessage, portMAX_DELAY)) {
            // send message to buffer
            sendMessageToBuffer(sendingMessage);
        }
    }
}

 

/*******************************************************************************
 End of File
 */
