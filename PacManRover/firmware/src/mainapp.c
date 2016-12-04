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
#include "motor_control_public.h"

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

// Sending a debug message
void sendDebugMessage(unsigned int val)
{
    messageStructure newMessage;
    // Deconstruct
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = mainappData.counter;
    newMessage.messageType = DEBUG; // will need to change for arbitration in message type
    newMessage.messageSize = sizeof(unsigned int);
    newMessage.messageContent[0] = (val & 0xFF000000) >> 24;
    newMessage.messageContent[1] = (val & 0x00FF0000) >> 16;
    newMessage.messageContent[2] = (val & 0x0000FF00) >> 8;
    newMessage.messageContent[3] = (val & 0x000000FF);
    // Increment and limit the value
    ++mainappData.counter;
    mainappData.counter %= 256;
    // Send
    sendToTXQueue(newMessage);
}

void sendSensorMessage(char* val)
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = mainappData.counter;
    newMessage.messageType = PACMAN_SENSOR;
    newMessage.messageSize = 4;
    // MSB in index 0
    newMessage.messageContent[0] = val[0];
    newMessage.messageContent[1] = val[1];
    newMessage.messageContent[2] = val[2];
    newMessage.messageContent[3] = val[3];
    // Increment and limit the value
    ++mainappData.counter;
    mainappData.counter %= 256;
    // Send
    sendToTXQueue(newMessage);
}

void sendCompleteMessage(void)
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = mainappData.counter;
    newMessage.messageType = PACMAN_ROVER_COMPLETE;
    newMessage.messageSize = 4;
    newMessage.messageContent[0] = 0x01;
    newMessage.messageContent[1] = 0x02;
    newMessage.messageContent[2] = 0x03;
    newMessage.messageContent[3] = 0x04;
    // Increment and limit the value
    ++mainappData.counter;
    mainappData.counter %= 256;
    // Send
    sendToTXQueue(newMessage);
}

void sendGameOverMessage(void)
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = mainappData.counter;
    newMessage.messageType = GAME_OVER;
    newMessage.messageSize = 4;
    newMessage.messageContent[0] = 0x00;
    newMessage.messageContent[1] = 0x00;
    newMessage.messageContent[2] = 0x00;
    newMessage.messageContent[3] = 0x64;
    // Increment and limit the value
    ++mainappData.counter;
    mainappData.counter %= 256;
    // Send
    sendToTXQueue(newMessage);
}

// Sends a message internally whenever the sensor sees all black
void sendExternalStopMessage()
{
    messageStructure newMessage;
    // Deconstruct
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
    newMessage.messageType = PACMAN_COMMAND; // will need to change for arbitration in message type
    newMessage.messageSize = sizeof(unsigned int);
    newMessage.messageContent[0] = FORWARD;
    newMessage.messageContent[1] = 0;
    newMessage.messageContent[2] = 0;
    newMessage.messageContent[3] = 1;
    // Send
    sendToMotorControlQueue(newMessage);
}

void convertAndSendSensorData(messageStructure oldMessage)
{
    messageStructure sendingMessage;
    sendingMessage.sender = MY_SENDER;
    sendingMessage.messageNumber = mainappData.counter;
    sendingMessage.messageType = PACMAN_SENSOR;
    sendingMessage.messageSize = 4;
    sendingMessage.messageContent[0] = oldMessage.messageContent[0];
    sendingMessage.messageContent[1] = oldMessage.messageContent[1];
    sendingMessage.messageContent[2] = oldMessage.messageContent[2];
    sendingMessage.messageContent[3] = oldMessage.messageContent[3];
    // Increment and limit the value
    ++mainappData.counter;
    mainappData.counter %= 256;
    // Send
    sendToTXQueue(sendingMessage);
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
    mainappData.counter = 0;
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

    // Testing for game-over to show for milestone 2
    int test = 0;
    
    PLIB_USART_Enable(USART_ID_1);

    while(1) {
        if(xQueueReceive(mainappData.mainQueue, &tempMsg, portMAX_DELAY)) {
            // Filter messages
            if(tempMsg.messageType == INITIAL_ORDER) {
                // Start up ADC when it becomes pertinent
                //DRV_ADC_Open();
                startSensing();
            }
            else if(tempMsg.messageType == PACMAN_COMMAND) {
                sendToMotorControlQueue(tempMsg);
            }
            else if(tempMsg.messageType == PACMAN_ROVER_COMPLETE) {
                // send from internally (motor control) outwards
                sendCompleteMessage();
            }
            else if(tempMsg.messageType == DEBUG) {
                // Filter out content
                //sendDebugMessage(30);
                sendToTXQueue(tempMsg);
            }
            else if(tempMsg.messageType == GAME_OVER) {
                sendExternalStopMessage();
                // Filter out content
                sendGameOverMessage();
            }
            else if(tempMsg.messageType == PACMAN_SENSOR) {
                convertAndSendSensorData(tempMsg);
            }
        }
    }
}

 

/*******************************************************************************
 End of File
 */
