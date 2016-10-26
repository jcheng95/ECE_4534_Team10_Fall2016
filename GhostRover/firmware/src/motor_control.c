/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motor_control.c

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

#include "motor_control.h"

#include "motor_control_public.h"

#include "peripheral/oc/plib_oc.h"


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

MOTOR_CONTROL_DATA motor_controlData;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t sendToMotorControlQueue(messageStructure msg)
{
    return xQueueSend(motor_controlData.motorControlQueue, &msg, portMAX_DELAY);
}

BaseType_t sendToMotorControlQueueFromISR(messageStructure msg)
{
    return xQueueSendFromISR(motor_controlData.motorControlQueue, &msg, 0);
}

void sendIRSensorMessage(char val)
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
    newMessage.messageType = GHOST_SENSOR;
    newMessage.messageSize = 4;
    // MSB in index 0
    newMessage.messageContent[0] = val;
    newMessage.messageContent[1] = 0;
    newMessage.messageContent[2] = 0;
    newMessage.messageContent[3] = 0;
    // Send
    sendToMainAppQueue(newMessage);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void rightMotorForward(int forward){
    if(forward)//forward
    {
        SYS_PORTS_PinClear ( PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14 );
    }
    else//backwards
    {
        SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14 );
    }
}

void leftMotorForward(int forward){
    if(forward)//forward
    {
        SYS_PORTS_PinClear ( PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1 );
    }
    else//backwards
    {
        SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1 );
    } 
}

void checkMessageContents(char direction){
    if (direction == FORWARD)
    {
        leftMotorForward(1);
        rightMotorForward(1);
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, 9500);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, 9500);
    }
    
    else if (direction == RIGHT)
    {
        leftMotorForward(1);
        rightMotorForward(0);
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, 9500);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, 9500);
    }
    
    else if (direction == LEFT)
    {
        leftMotorForward(0);
        rightMotorForward(1);
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, 9500);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, 9500);
    }
    
    else if (direction == BACKWARD)
    {
        leftMotorForward(0);
        rightMotorForward(0);
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, 9500);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, 9500);
    }
}

void activateTimer(){
    initializeTimer();
}
                            
void irLEDOn()
{
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_10 );
}

void ioLineOutput()
{
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_7 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_6 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_5 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_4 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_3 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_2 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_1 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_E, PORTS_BIT_POS_0 );    
}

void ioLineInput()
{
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_7 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_6 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_5 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_4 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_3 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_2 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_1 );
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_E, PORTS_BIT_POS_0 );  
}

void ioLineDrive()
{
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7 );
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6 );
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5 );
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4 );
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3 );
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2 );
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1 );
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0 );

}

void ioPinRead()
{
    char sensor1 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7);
    char sensor2 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6);
    char sensor3 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5);
    char sensor4 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4);
    char sensor5 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3);
    char sensor6 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2);
    char sensor7 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
    char sensor8 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0);
    
    motor_controlData.lfData = (sensor1 << 7) | 
                               (sensor2 << 6) |
                               (sensor3 << 5) |
                               (sensor4 << 4) |
                               (sensor5 << 3) |
                               (sensor6 << 2) |
                               (sensor7 << 1) |
                               (sensor8);    
}

void irLEDOff()
{
    SYS_PORTS_PinClear ( PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_10 );
}

void irSensor(){
    switch(motor_controlData.fsm){
            case irOn:
                irLEDOn();
                motor_controlData.fsm = ioOut;
                break;
            case ioOut:
                ioLineOutput();
                ioLineDrive();
                motor_controlData.fsm = ioIn;
                break;
            case ioIn:
                ioLineInput();
                motor_controlData.fsm = ioRead;
                break;
            case ioRead:
                ioPinRead();
                sendIRSensorMessage(motor_controlData.lfData);
                irLEDOff();
                motor_controlData.fsm = irOn;
                break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MOTOR_CONTROL_Initialize ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Initialize ( void )
{
    motor_controlData.motorControlQueue = xQueueCreate(16, sizeof(messageStructure));
    motor_controlData.fsm = irOn;
    //PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
    //PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
}


/******************************************************************************
  Function:
    void MOTOR_CONTROL_Tasks ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Tasks ( void ){

    PLIB_OC_Enable(OC_ID_1);
    PLIB_OC_Enable(OC_ID_2);
    DRV_TMR0_Start();
    
    //PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
    //PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
    
    messageStructure tempMsg;
    //rightMotorForward(1);
    //leftMotorForward(1);
    
    while(1) {
        // Do encoder readings and powering motors
        //xqueue recieve
        if(xQueueReceive(motor_controlData.motorControlQueue, &tempMsg, portMAX_DELAY)) 
        {
            if (tempMsg.messageContent[3] == 1)
            {
                PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
                PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
            }
            else
            {
                checkMessageContents(tempMsg.messageContent[0]);
            }
                
        }

    }        // Do encoder readings and powering motors

    
    
    
}

 

/*******************************************************************************
 End of File
 */
