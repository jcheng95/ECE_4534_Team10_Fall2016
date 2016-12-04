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

void sendInternalStopMessage()
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
    newMessage.messageType = GHOST_COMMAND;
    newMessage.messageSize = 4;
    newMessage.messageContent[0] = 0x01;
    newMessage.messageContent[1] = 0x00;
    newMessage.messageContent[2] = 0x00;
    newMessage.messageContent[3] = 0x01;
    // Send
    sendToMotorControlQueue(newMessage);
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

void sendDebugMessage()
{
    messageStructure newMessage;
    // Deconstruct
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
    newMessage.messageType = DEBUG; // will need to change for arbitration in message type
    newMessage.messageSize = sizeof(unsigned int);
    newMessage.messageContent[0] = 0;
    newMessage.messageContent[1] = 0;
    newMessage.messageContent[2] = (motor_controlData.rightMotor.pwm & 0xFF00) >> 8;
    newMessage.messageContent[3] = (motor_controlData.rightMotor.pwm & 0x00FF);
    // Send
    sendToTXQueue(newMessage);
}

/*void sendInternalCompleteMessage()
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
    sendToMainAppQueue(newMessage);
}*/

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

void bothMotorsStop (){
    motor_controlData.leftMotor = motor_controlData.stopValues;
    motor_controlData.rightMotor = motor_controlData.stopValues;
    motor_controlData.roverDirection = stop;
}

void bothMotorsFullSpeed(){
    motor_controlData.leftMotor = motor_controlData.fullSpeedValues;
    motor_controlData.rightMotor = motor_controlData.fullSpeedValues;
}

void activateTimer(){
    initializeSensorTimer();
}
                            
void irLEDOn()
{
    SYS_PORTS_PinSet ( PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_10 );
}

void convertToWeighted()
{
    if (motor_controlData.lfData == 0x03) {
        motor_controlData.weighted_lfData = -3;
    }
    else if (motor_controlData.lfData == 0x06) {
        motor_controlData.weighted_lfData = -2;
    }
    else if (motor_controlData.lfData == 0x0c) {
        motor_controlData.weighted_lfData = -1;
    }
    else if (motor_controlData.lfData == 0x18) {
        motor_controlData.weighted_lfData = 0;
    }
    else if (motor_controlData.lfData == 0x30) {
        motor_controlData.weighted_lfData = 1;
    }
    else if (motor_controlData.lfData == 0x60) {
        motor_controlData.weighted_lfData = 2;
    }
    else if (motor_controlData.lfData == 0xc0) {
        motor_controlData.weighted_lfData = 3;
    }
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
    unsigned char sensor1 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7);
    unsigned char sensor2 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6);
    unsigned char sensor3 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5);
    unsigned char sensor4 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4);
    unsigned char sensor5 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3);
    unsigned char sensor6 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2);
    unsigned char sensor7 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
    unsigned char sensor8 = SYS_PORTS_PinRead ( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0);
    
    motor_controlData.lfData = (sensor1 << 7) | 
                               (sensor2 << 6) |
                               (sensor3 << 5) |
                               (sensor4 << 4) |
                               (sensor5 << 3) |
                               (sensor6 << 2) |
                               (sensor7 << 1) |
                               (sensor8);
    convertToWeighted();
}

void irLEDOff()
{
    SYS_PORTS_PinClear ( PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_10 );
}

void checkForIntersection()
{
    if(motor_controlData.roverDirection == forward || motor_controlData.roverDirection == backward) {
        if((motor_controlData.lfData & 0x3C) == 0x3C) {
            bothMotorsStop();
            PLIB_OC_PulseWidth16BitSet(OC_ID_2, motor_controlData.leftMotor.pwm);
            PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rightMotor.pwm);
            sendInternalCompleteMessage();
        }
    }
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
                if((motor_controlData.roverDirection == forward || motor_controlData.roverDirection == backward) && motor_controlData.intersectionCounter != 10) {
                    ++motor_controlData.intersectionCounter;
                }
                else {
                    checkForIntersection();
                }
                //sendIRSensorMessage(motor_controlData.lfData);
                irLEDOff();
                motor_controlData.fsm = irOn;
                break;
    }
}

void checkMessageContents(char direction){
    if (direction == FORWARD)
    {
        motor_controlData.roverDirection = forward;
        leftMotorForward(1);
        rightMotorForward(1);
        bothMotorsFullSpeed();
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rightMotor.pwm);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, motor_controlData.leftMotor.pwm);
    }
    
    else if (direction == RIGHT)
    {
        motor_controlData.roverDirection = right;
        leftMotorForward(1);
        rightMotorForward(0);
        bothMotorsFullSpeed();
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rightMotor.pwm);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, motor_controlData.leftMotor.pwm);
    }
    
    else if (direction == LEFT)
    {
        motor_controlData.roverDirection = left;
        leftMotorForward(0);
        rightMotorForward(1);
        bothMotorsFullSpeed();
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rightMotor.pwm);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, motor_controlData.leftMotor.pwm);
    }
    
    else if (direction == BACKWARD)
    {
        motor_controlData.roverDirection = backward;
        leftMotorForward(0);
        rightMotorForward(0);
        bothMotorsFullSpeed();
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rightMotor.pwm);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, motor_controlData.leftMotor.pwm);
    }
}

void checkCounter(){
    if (motor_controlData.roverDirection == left || motor_controlData.roverDirection == right)
    {
        if (motor_controlData.tickCounter >= TICKS){
            //bothMotorsStop();
            PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
            PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
            motor_controlData.roverDirection = stop;
            motor_controlData.tickCounter = 0;
            sendInternalCompleteMessage();
        }
        else{
            ++motor_controlData.tickCounter;
        }
    }/*
    if(motor_controlData.tickCounter > 0) {
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
        PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
        motor_controlData.tickCounter = 0;
    }
    else {
        ++motor_controlData.tickCounter;
    }*/
}

void motorPIDController(){//MOTOR_CONTROL_VALUES& motor){
    /*motor->error = 24 - (motor->encoderTicks - motor->prevEncoderTicks);
    motor->prevEncoderTicks = motor->encoderTicks;
    motor->integral = motor->integral + (motor->error / 20);
    int16_t derivative = (motor->error - motor->prevError)*20;
    int16_t output = (0)*motor->error + (0)*motor->integral + (0)*derivative;
    motor->prevError = motor->error;
    motor->pwm = 9500 + output;*/
    //Change the OC PWM width <- Do this in a encompassing function that calls the PID controller for each motor <- function called as the callback for a software timer
}

int16_t sensorLeftPIDController()//MOTOR_CONTROL_VALUES& motor)//, BaseType_t left, BaseType_t right)
{
    int16_t error = 0 - motor_controlData.weighted_lfData;
    motor_controlData.leftMotor.sensorError = 0 - motor_controlData.weighted_lfData;
    motor_controlData.leftMotor.sensorIntegral = motor_controlData.leftMotor.sensorIntegral + (error / 20);
    int16_t derivative = (motor_controlData.leftMotor.prevSensorError - error)*20;
    int16_t output = (2250)*error; + (600)*motor_controlData.leftMotor.sensorIntegral + (902)*derivative;
    motor_controlData.leftMotor.prevSensorError = motor_controlData.leftMotor.sensorError;
    return output;
    //sendIRSensorMessage(motor_controlData.lfData);
}

int16_t sensorRightPIDController()//MOTOR_CONTROL_VALUES& motor)//, BaseType_t left, BaseType_t right)
{
    int16_t error = 0 - motor_controlData.weighted_lfData;
    motor_controlData.rightMotor.sensorError = 0 - motor_controlData.weighted_lfData;
    motor_controlData.rightMotor.sensorIntegral = motor_controlData.rightMotor.sensorIntegral + (error / 20);
    int16_t derivative = (motor_controlData.rightMotor.prevSensorError - error)*20;
    int16_t output = (2000)*error; + (518)*motor_controlData.rightMotor.sensorIntegral + (763)*derivative;
    motor_controlData.rightMotor.prevSensorError = motor_controlData.rightMotor.sensorError;
    return output;
    //sendIRSensorMessage(motor_controlData.lfData);
}

void incrementLeftEncoder(){
    ++motor_controlData.leftMotor.encoderTicks;
}

void incrementRightEncoder(){
    ++motor_controlData.rightMotor.encoderTicks;
}

void motorCaller(){
    if(motor_controlData.leftMotor.pwm == 0 && motor_controlData.rightMotor.pwm == 0) {
        //return;
        // Do absolutely nothing
    }
    else {
        /*motorPIDController(motor_controlData.leftMotor);
        motorPIDController(motor_controlData.rightMotor);*/
        if (motor_controlData.roverDirection == forward || motor_controlData.roverDirection == backward) {
            motor_controlData.leftMotor.pwm = 9500 + sensorLeftPIDController();//motor_controlData.leftMotor);//, pdTRUE, pdFALSE);
            if(motor_controlData.leftMotor.pwm >= 10000) {
                motor_controlData.leftMotor.pwm = 10000;
            }
            else if(motor_controlData.leftMotor.pwm <= 0) {
                motor_controlData.leftMotor.pwm = 0;
            }
            motor_controlData.rightMotor.pwm = 9500 - sensorRightPIDController();//motor_controlData.rightMotor);//, pdFALSE, pdTRUE);
            if(motor_controlData.rightMotor.pwm >= 10000) {
                motor_controlData.rightMotor.pwm = 10000;
            }
            else if(motor_controlData.rightMotor.pwm <= 0) {
                motor_controlData.rightMotor.pwm = 0;
            }
            //sendDebugMessage();
            PLIB_OC_PulseWidth16BitSet(OC_ID_2, motor_controlData.leftMotor.pwm);
            PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rightMotor.pwm);
        }
    }
}

void setupValues(MOTOR_CONTROL_VALUES *motor, int16_t newPWM)
{
    motor->encoderTicks = 0;
    motor->error = 0;
    motor->integral = 0;
    motor->prevEncoderTicks = 0 ;
    motor->prevError = 0;
    
    motor->sensorError = 0;
    motor->prevSensorError = 0;
    motor->sensorIntegral = 0;
    motor->pwm = newPWM;
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
    motor_controlData.roverDirection = stop;
    motor_controlData.motorControlQueue = xQueueCreate(16, sizeof(messageStructure));
    motor_controlData.fsm = irOn;
    motor_controlData.tickCounter = 0;
    motor_controlData.lfData = 0;
    motor_controlData.weighted_lfData = 0;
    setupValues(&motor_controlData.fullSpeedValues, 9500);
    setupValues(&motor_controlData.stopValues, 0);
    motor_controlData.leftMotor = motor_controlData.stopValues;
    motor_controlData.rightMotor = motor_controlData.stopValues;
    motor_controlData.intersectionCounter = 5;
    /*motor_controlData.leftMotor->pwm = 0;
    motor_controlData.leftMotor->encoderTicks = 0;
    motor_controlData.leftMotor->error = 0;
    motor_controlData.leftMotor->integral = 0;
    motor_controlData.leftMotor->prevError = 0;
    motor_controlData.leftMotor->prevEncoderTicks = 0;
    motor_controlData.rightMotor->pwm = 0;
    motor_controlData.rightMotor->encoderTicks = 0;
    motor_controlData.rightMotor->error = 0;
    motor_controlData.rightMotor->integral = 0;
    motor_controlData.rightMotor->prevError = 0;
    motor_controlData.rightMotor->prevEncoderTicks = 0;*/
}


/******************************************************************************
  Function:
    void MOTOR_CONTROL_Tasks ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Tasks ( void )
{
    PLIB_OC_Enable(OC_ID_1);//right motor
    //m1 = oc1 = sa1 = right motor = pin2 = int1 
    PLIB_OC_Enable(OC_ID_2);//left motor
    //m2 = oc2 = sa2 = left motor = pin7 = int2
    DRV_TMR0_Start();
    initializeMotorTimer();
    //PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
    //PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
    
    messageStructure tempMsg;
    rightMotorForward(1);
    leftMotorForward(1);
    
    while(1) {
        // Do encoder readings and powering motors
        //xqueue recieve
        if(xQueueReceive(motor_controlData.motorControlQueue, &tempMsg, portMAX_DELAY)) 
        {
            if (tempMsg.messageContent[3] == 1)
            {
                //if the client recieves a j command, set pwm width to zero to stop motors
                bothMotorsStop();
                PLIB_OC_PulseWidth16BitSet(OC_ID_2, motor_controlData.leftMotor.pwm);
                PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rightMotor.pwm);
                motor_controlData.roverDirection = stop;
                /*PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
                PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);*/
            }
            else
            {
                checkMessageContents(tempMsg.messageContent[0]);
                if (motor_controlData.roverDirection == forward || motor_controlData.roverDirection == backward)
                {
                    motor_controlData.intersectionCounter = 0;
                }
            } 
        }
    }        // Do encoder readings and powering motors
}

 

/*******************************************************************************
 End of File
 */
