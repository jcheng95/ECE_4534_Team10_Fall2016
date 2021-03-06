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
#include "motor_control.h"
#include "adc_app.h"
#include "uart_rx.h"
#include "uart_tx.h"
#include "system_definitions.h"

#include <queue.h>
#include "common.h"
#include "tx_buffer_public.h"
#include "adc_app_public.h"
#include "motor_control_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
QueueHandle_t bufferQueue;
//unsigned int curVal;

void initializeBufferQueue()
{
    bufferQueue = xQueueCreate(MAX_BUFFER_SIZE, 8);
}

void IntHandlerDrvAdc(void)
{
    int i;
    unsigned int curVal = 0;
    //Read data before clearing interrupt flag
    for(i = 0; i < MAX_SAMPLE_SIZE; i++)
        curVal += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);
    curVal = curVal / MAX_SAMPLE_SIZE;
    
    sendToADCAppQueueFromISR(curVal);
    
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
}

BaseType_t sendToTXBufferQueue(char msg)
{
    PLIB_INT_SourceEnable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
    return xQueueSend(bufferQueue, &msg, portMAX_DELAY);
}

BaseType_t sendToTXBufferQueueFromISR(char msg)
{
    PLIB_INT_SourceEnable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
    return xQueueSendFromISR(bufferQueue, &msg, 0);
}

void sendInternalCompleteMessage()
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
    newMessage.messageType = GHOST_ROVER_COMPLETE;
    newMessage.messageSize = 4;
    // MSB in index 0
    newMessage.messageContent[0] = 0x01;
    newMessage.messageContent[1] = 0x02;
    newMessage.messageContent[2] = 0x03;
    newMessage.messageContent[3] = 0x04;
    // Send
    sendToMainAppQueueFromISR(newMessage);
}

void IntHandlerExternalInterruptInstance0(void)//right motor
{
    //bothMotorsStop();
    //this encoder reading is currently not working
    //NEED TO REPAIR
    //incrementRightEncoder();
    //checkCounter();
    //sendInternalCompleteMessage();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);
}
void IntHandlerExternalInterruptInstance1(void)//left motor
{
    //bothMotorsStop();
    //incrementLeftEncoder();
    checkCounter();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_2);
}
    
void IntHandlerDrvTmrInstance0(void)
{
    //sendToADCAppQueueFromISR(curVal);
    // Clear the interrupt flag
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
 void IntHandlerDrvUsartInstance0(void)
{
    char sendByte;
    char recvByte;
    
    // Interrupt driven by the receive
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE)) {
        while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
            recvByte = PLIB_USART_ReceiverByteReceive(USART_ID_1);
            sendToRXQueueFromISR(recvByte);
        }
    }
    
    // Interrupt driven by the transmit
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)) {
        while(!PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) {
            if(xQueueReceiveFromISR(bufferQueue, &sendByte, 0)) {
                // Sending because there is something in the buffer
                PLIB_USART_TransmitterByteSend(USART_ID_1, sendByte);
            }
            else {
                // Disable the interrupt because there is nothing to send
                PLIB_INT_SourceDisable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
                break;
            }
        }
    }
    
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
}
 
 
 

 

 

 

 

 
 
 
/*******************************************************************************
 End of File
*/

