/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    adc_app.c

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

#include "adc_app.h"
#include "adc_app_public.h"

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

ADC_APP_DATA adc_appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t sendToADCQueue(unsigned short val)
{
    return xQueueSend(adc_appData.adcQueue, &val, portMAX_DELAY);
}

BaseType_t sendToADCQueueFromISR(unsigned short val)
{
    return xQueueSendFromISR(adc_appData.adcQueue, &val, 0);
}

void activateReadySignal(void)
{
    adc_appData.ready = true;
}

void addToSampleValue(ADC_SAMPLE val)
{
    adc_appData.curVal += val;
}

void averageSample(void)
{
    adc_appData.curVal = adc_appData.curVal / MAX_SAMPLE_SIZE;
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
    newMessage.messageContent[0] = (adc_appData.curVal & 0xFF000000) >> 24;
    newMessage.messageContent[1] = (adc_appData.curVal & 0x00FF0000) >> 16;
    newMessage.messageContent[2] = (adc_appData.curVal & 0x0000FF00) >> 8;
    newMessage.messageContent[3] = (adc_appData.curVal & 0x000000FF);
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
    void ADC_APP_Initialize ( void )

  Remarks:
    See prototype in adc_app.h.
 */

void ADC_APP_Initialize ( void )
{
    // Right now these values may not be right but they're placeholders
    adc_appData.adcQueue = xQueueCreate(16, sizeof(unsigned int));
    // Creating a private member to hold the current sensor value.
    adc_appData.curVal = 0;
    // Making sure the signal to be ready to read data is initially false
    adc_appData.ready = false;
}


/******************************************************************************
  Function:
    void ADC_APP_Tasks ( void )

  Remarks:
    See prototype in adc_app.h.
 */

void ADC_APP_Tasks ( void )
{
    // Enable the ADC
    DRV_ADC_Open();
    
    unsigned short recvVal;
    while(1) {
        // Reading from ADC interrupt to send to main app queue
        if(adc_appData.ready == true) {
            // Value manipulation
            // Send out to mainapp for calculation (maybe)
            sendMessage();
            adc_appData.ready = false;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
