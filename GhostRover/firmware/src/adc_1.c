/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    adc_1.c

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

#include "adc_1.h"

#include "adc_1_public.h"

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

ADC_1_DATA adc_1Data;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t sendToADCAppQueue(unsigned int msg)
{
    return xQueueSend(adc_1Data.adcQueue, &msg, portMAX_DELAY);
}

BaseType_t sendToADCAppQueueFromISR(unsigned int msg)
{
    return xQueueSendFromISR(adc_1Data.adcQueue, &msg, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

// This function converts the results of the ADC read into human-readable centimeters
unsigned int convertToCentimeters(unsigned int val)
{
   unsigned int convertedResult;
   
   //convertedResult = (unsigned int)exp(((double)val - 1333.2) / -285.9);
   convertedResult = (unsigned int)(47457.4 / pow((double)val, 1.29534));
   return convertedResult;
}

// Sends sensor data, with the data type int, to mainapp
void sendSensorData(unsigned int val)
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
    newMessage.messageType = GHOST_SENSOR;
    newMessage.messageSize = 4;
    // MSB in index 0
    newMessage.messageContent[0] = (val & 0xFF000000) >> 24;
    newMessage.messageContent[1] = (val & 0x00FF0000) >> 16;
    newMessage.messageContent[2] = (val & 0x0000FF00) >> 8;
    newMessage.messageContent[3] = (val & 0x000000FF);
    
    sendToMainAppQueue(newMessage);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ADC_1_Initialize ( void )

  Remarks:
    See prototype in adc_1.h.
 */

void ADC_1_Initialize ( void )
{
    adc_1Data.adcQueue = xQueueCreate(16, sizeof(unsigned int));
}


/******************************************************************************
  Function:
    void ADC_1_Tasks ( void )

  Remarks:
    See prototype in adc_1.h.
 */

void ADC_1_Tasks ( void )
{
    unsigned int tempMsg;
    
    // Enabling the ADC for sensor data
    //DRV_ADC_Open();
    
    // Activates the timer driver
    // Required in order for the timer to work
    //DRV_TMR0_Start();
    
    while(1) {
        if(xQueueReceive(adc_1Data.adcQueue, &tempMsg, portMAX_DELAY)) {
            // Process and send to TX queue
            sendSensorData(convertToCentimeters(tempMsg));
        }
    }
}

 

/*******************************************************************************
 End of File
 */
