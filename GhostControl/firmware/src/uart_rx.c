/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uart_rx.c

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

#include "uart_rx.h"

#include "uart_rx_public.h"

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

UART_RX_DATA uart_rxData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t sendToRXQueue(messageStructure msg)
{
    xQueueSend(uart_rxData.rxQueue, &msg, portMAX_DELAY);
}

BaseType_t sendToRXQueueFromISR(messageStructure msg)
{
    xQueueSendFromISR(uart_rxData.rxQueue, &msg, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void filterAndSend(messageStructure msg)
{
    switch(msg.messageType) {
        case INITIAL_ORDER:
            sendToMainAppQueue(msg);
            break;
        case GHOST_SENSOR:
            sendToMainAppQueue(msg);
            break;
        case GHOST_ROVER_COMPLETE:
            sendToMainAppQueue(msg);
            break;
        case DEBUG:
            sendToMainAppQueue(msg);
            break;
        case GAME_OVER:
            sendToMainAppQueue(msg);
            break;
        default:
            break;
    }
}

messageStructure stringToMessage(unsigned char arr[MAX_TOTAL_SIZE])
{
    messageStructure converted;
    converted.sender = arr[1];
    converted.messageNumber = arr[2];
    converted.messageType = arr[3];
    converted.messageSize = arr[4];
    
    int i;
    for(i = 0; i < converted.messageSize; i++) {
        converted.messageContent[i] = arr[5 + i];
    }
    return converted;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UART_RX_Initialize ( void )

  Remarks:
    See prototype in uart_rx.h.
 */

void UART_RX_Initialize ( void )
{
    uart_rxData.rxQueue = xQueueCreate(MAX_BUFFER_SIZE, 8);
    uart_rxData.messageCounter = 0;
}


/******************************************************************************
  Function:
    void UART_RX_Tasks ( void )

  Remarks:
    See prototype in uart_rx.h.
 */

void UART_RX_Tasks ( void )
{
    unsigned char message[MAX_TOTAL_SIZE];
    messageStructure newMessage;
    
    while(1) {
        while(!xQueueReceive(uart_rxData.rxQueue, &message[uart_rxData.messageCounter], portMAX_DELAY));
        // Increment counter for next byte in the char array
        uart_rxData.messageCounter++;
        
        if(message[0] == START_BYTE) {
            if((uart_rxData.messageCounter == MAX_TOTAL_SIZE) && (message[MAX_TOTAL_SIZE - 1] == END_BYTE)) {
                // Process the new message
                newMessage = stringToMessage(message);
                filterAndSend(newMessage);
                // Reset for a new message
                uart_rxData.messageCounter = 0;
            } else if((uart_rxData.messageCounter == MAX_TOTAL_SIZE) && (message[MAX_TOTAL_SIZE - 1] != END_BYTE)) {
                // Restart searching for a message
                uart_rxData.messageCounter = 0;
            }
        } else {
            // Restart searching for a message
            uart_rxData.messageCounter = 0;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
