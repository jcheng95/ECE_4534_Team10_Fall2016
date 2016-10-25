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

#define LEFT_OSCILLATOR  OC_ID_2
#define RIGHT_OSCILLATOR OC_ID_1
/* Total period is (prescaler * timer value) but max duty cycle is only half the period */
/* Timer ticks are determined by (system period * prescaler)
   In my case, this would be (1 / 80 MHz) * 32 = 400 ns = 0.4 us
   So one timer 2 tick happens every 0.4 us */
#define PWM_MAX_VALUE     10000   // Dependent on timer 2 value
#define TEST_START_VALUE  9000
// Line-following sensor ideal value (center) = 0x18
#define IDEAL_SENSOR_VALUE  24
/* At a timer period of 500 ms, the ideal encoder value is 265, the Kp is 27 (or 28), the Ki is 210 */
// Approximately 1 tick every 1.887 ms which results in an ideal number of ticks for a 500 ms timer to be 264.957
// For a 50 ms timer, there would be 26.4957
#define IDEAL_ENCODER_VALUE 27
#define NINETY_DEGREES    620     // 620 ticks from the external interrupt indicates 90 degrees for a turn <- this approximately takes 1.17s with the PWM width at 9000
/* Gain parameters for PID controller - Encoders */
#define K_P         502
#define K_I         30//30
#define K_D         0
/* Gain parameters for PID controller - Line-following sensor */
#define SENSOR_K_P  0
#define SENSOR_K_I  0
#define SENSOR_K_D  0

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

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

// Sends a message internally whenever the sensor sees all black
void sendInternalStopMessage()
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

// Sends a movement completion message
void sendInternalCompleteMessage(void)
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
    newMessage.messageType = PACMAN_ROVER_COMPLETE;
    newMessage.messageSize = 4;
    newMessage.messageContent[0] = 0x01;
    newMessage.messageContent[1] = 0x02;
    newMessage.messageContent[2] = 0x03;
    newMessage.messageContent[3] = 0x04;
    // Send
    sendToMainAppQueueFromISR(newMessage);
}

// Performs secondary PID controller to cascade onto the main motor control loop
void lineFollowingCorrection()
{
    // Stopping the motors if you've reached an intersection
    if((motor_controlData.sensorValues & 0xFF) == 0xFF) {
        sendInternalStopMessage();
    }
    // Anything else
    else {
        // Not centered
        /* Reliant on the two center sensors. if one of the center sensors is off, immediately a correction needs to be made */
        if(!(motor_controlData.sensorValues & 0xFF == 0x18)) {
            // minor corrections to the PWM
            if(motor_controlData.sensorValues & 0xFF == 0x01) {
                // Far off to the left
            }
            else if(motor_controlData.sensorValues & 0xFF == 0x03) {
                // Off to the left
            }
            else if(motor_controlData.sensorValues & 0xFF == 0x06) {
                // Slightly off to the left
            }
            else if(motor_controlData.sensorValues & 0xFF == 0x0D) {
                // Very slightly off to the left
            }
            else if(motor_controlData.sensorValues & 0xFF == 0x80) {
                // Far off to the right
            }
            else if(motor_controlData.sensorValues & 0xFF == 0xD0) {
                // Off to the right
            }
            else if(motor_controlData.sensorValues & 0xFF == 0x60) {
                // Slightly off to the right
            }
            else if(motor_controlData.sensorValues & 0xFF == 0x30) {
                // Very slightly off to the right
            }
        }
    }
}

// Changes whether or not the LED is on or off
void setLEDState(BaseType_t on)
{
    if(on) {
        // Setting the LED for the line-following sensor high
        SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    }
    else {
        // Setting the LED for the line-following sensor high
        SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    }
}

// Setting the GPIO pins for the line-following sensor pins to output
void setSensorPinsOutput()
{
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, PORTS_BIT_POS_7);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, PORTS_BIT_POS_6);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, PORTS_BIT_POS_5);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, PORTS_BIT_POS_4);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, PORTS_BIT_POS_3);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, PORTS_BIT_POS_2);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, PORTS_BIT_POS_1);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, PORTS_BIT_POS_0);
}

// Driving the GPIO pins for the line-following sensor high
void setSensorPinsHigh()
{
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0);
}

// Setting the GPIO pins for the line-following sensor pins to input
void setSensorPinsInput()
{
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_INPUT,PORT_CHANNEL_E, PORTS_BIT_POS_7);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_INPUT,PORT_CHANNEL_E, PORTS_BIT_POS_6);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_INPUT,PORT_CHANNEL_E, PORTS_BIT_POS_5);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_INPUT,PORT_CHANNEL_E, PORTS_BIT_POS_4);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_INPUT,PORT_CHANNEL_E, PORTS_BIT_POS_3);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_INPUT,PORT_CHANNEL_E, PORTS_BIT_POS_2);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_INPUT,PORT_CHANNEL_E, PORTS_BIT_POS_1);
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_INPUT,PORT_CHANNEL_E, PORTS_BIT_POS_0);
}

// Reading the pins for the line-following sensor
void readSensorPins()
{
    unsigned char sensorEight = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7);
    unsigned char sensorSeven = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6);
    unsigned char sensorSix = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5);
    unsigned char sensorFive = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4);
    unsigned char sensorFour = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3);
    unsigned char sensorThree = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2);
    unsigned char sensorTwo = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
    unsigned char sensorOne = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0);
    motor_controlData.sensorValues = ((sensorEight << 7) | (sensorSeven << 6) | (sensorSix << 5) | (sensorFive << 4) |
                                      (sensorFour << 3) | (sensorThree << 2) | (sensorTwo << 1) | (sensorOne));
    
    lineFollowingCorrection();
}

// Sending the sensor data as a sensor data message
void sendSensorData()
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
    newMessage.messageType = PACMAN_SENSOR;
    newMessage.messageSize = 4;
    // MSB in index 0
    newMessage.messageContent[0] = motor_controlData.sensorValues;
    newMessage.messageContent[1] = 0;
    newMessage.messageContent[2] = 0;
    newMessage.messageContent[3] = 0;
    // Send
    sendToMainAppQueue(newMessage);
}

// Enables pin 78 (RG1 - corresponds to pin 34 on motor shield) to drive the left motor to move forward or backward
void makeLeftMotorForward(BaseType_t forward)
{
    // If the intent is to go forward, set the pin to hold the 0 value
    if(forward) {
        SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
    else {
        SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
}

// Enables pin 4 (RC14 - corresponds with pin 4 on motor shield) to drive the right motor to move forward or backward
void makeRightMotorForward(BaseType_t forward)
{
    if(forward) {
        SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
    else {
        SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
}

// Initializes default rover values
void setInitialMotorValues(Motor* motor, uint16_t newPWMValue, uint16_t newIdealEncoderValue)
{
    // Initializing PWM width and ideal encoder value for controlling the motor movement and the PID controller
    motor->pwmWidth = newPWMValue;
    motor->idealEncoderValue = newIdealEncoderValue;
    
    // Defaulting all other values to 0 because they are required to start at 0
    motor->encoderValue = 0;
    motor->previousEncoderValue = 0;
    motor->integral = 0;
    motor->previousError = 0;
    
}

// Setting both motors with max values
void setMotorAtMaxSpeed()
{
    motor_controlData.leftMotor = &motor_controlData.leftMotorMaxValues;
    motor_controlData.rightMotor = &motor_controlData.rightMotorMaxValues;
}

// Stopping both motors
void stopMotor()
{
    motor_controlData.leftMotor = &motor_controlData.stopMotorValues;
    motor_controlData.rightMotor = &motor_controlData.stopMotorValues;
}

void incrementLeftEncoder()
{
    ++motor_controlData.leftMotor->encoderValue;
}

void incrementRightEncoder()
{
    ++motor_controlData.rightMotor->encoderValue;
}

void checkForStop(BaseType_t leftMotor, BaseType_t rightMotor)
{
    if(leftMotor && ((motor_controlData.states == movingLeft) || (motor_controlData.states == movingRight))) {
        if(motor_controlData.turnCounter >= motor_controlData.turnStopValue) {
            stopMotor();
            sendInternalCompleteMessage();
            motor_controlData.states = stopMoving;
            motor_controlData.turnCounter = 0;
            // Set the PWM value to affect the speed of the rover
            PLIB_OC_PulseWidth16BitSet(LEFT_OSCILLATOR, motor_controlData.leftMotor->pwmWidth);
            PLIB_OC_PulseWidth16BitSet(RIGHT_OSCILLATOR, motor_controlData.rightMotor->pwmWidth);
        }
        else {
            ++motor_controlData.turnCounter;
        }
    }
    else if(rightMotor && ((motor_controlData.states == movingLeft) || (motor_controlData.states == movingRight))) {
        if(motor_controlData.turnCounter >= motor_controlData.turnStopValue) {
            stopMotor();
            sendInternalCompleteMessage();
            motor_controlData.states = stopMoving;
            motor_controlData.turnCounter = 0;
            // Set the PWM value to affect the speed of the rover
            PLIB_OC_PulseWidth16BitSet(LEFT_OSCILLATOR, motor_controlData.leftMotor->pwmWidth);
            PLIB_OC_PulseWidth16BitSet(RIGHT_OSCILLATOR, motor_controlData.rightMotor->pwmWidth);
        }
        else {
            ++motor_controlData.turnCounter;
        }
    }
}

// Determines the movement of the rover based on the direction of the command
void controlMotor(char direction) {
    switch(direction) {
        case FORWARD:
            // Both pins need to be set to 0 to move forward
            makeLeftMotorForward(pdTRUE);
            makeRightMotorForward(pdTRUE);
            setMotorAtMaxSpeed();
            motor_controlData.states = movingForward;
            break;
        case BACKWARD:
            // Both pins need to be set to 1 to move backward
            makeLeftMotorForward(pdFALSE);
            makeRightMotorForward(pdFALSE);
            setMotorAtMaxSpeed();
            motor_controlData.states = movingBackward;
            break;
        case LEFT:
            // To turn in place, the left motor must be in the opposite direction as the right motor
            // The left motor needs to move backward and the right motor needs to move forward
            // Reverse the expected logic
            makeLeftMotorForward(pdFALSE);
            makeRightMotorForward(pdTRUE);
            setMotorAtMaxSpeed();
            motor_controlData.states = movingLeft;
            break;
        case RIGHT:
            // To turn in place, the left motor must be in the opposite direction as the right motor
            // The left motor needs to move forward and the right motor needs to move backward
            // Reverse the expected logic
            makeLeftMotorForward(pdTRUE);
            makeRightMotorForward(pdFALSE);
            setMotorAtMaxSpeed();
            motor_controlData.states = movingRight;
            break;
        default:
            // By default, don't let the rover move
            stopMotor();
            motor_controlData.states = stopMoving;
            break;
    }
    // Set the PWM value to affect the speed of the rover
    PLIB_OC_PulseWidth16BitSet(LEFT_OSCILLATOR, motor_controlData.leftMotor->pwmWidth);
    PLIB_OC_PulseWidth16BitSet(RIGHT_OSCILLATOR, motor_controlData.rightMotor->pwmWidth);
}

// PID Controller Algorithm
// Approximately 1 tick every 1.887 ms
// For a 500 ms timer, there would be ideally 264.957 ticks
void pidAlgorithm(Motor* motor, uint16_t gainP, uint16_t gainI, uint16_t gainD)
{
    // Measured value = current encoder value - previous encoder value
    uint16_t measuringPoint = motor->encoderValue - motor->previousEncoderValue;
    // Updating the previous encoder value for the next timer call
    motor->previousEncoderValue = motor->encoderValue;
    // Error = setpoint - measured value
    // Need to figure out what the appropriate encoder value is
    /* This way, the setpoint and the measured value are based on the last timer call */
    uint16_t error = motor->idealEncoderValue - measuringPoint;
    // Integral = integral + error * dt
    // dt for the integral and derivative is 500 ms
    motor->integral += (error / 2);
    // Derivative = (previous error - current error) / dt
    uint16_t derivative = (motor->previousError - error) * 2;
    motor->previousError = error;
    // Setting the output to the PWM for correction
    motor->pwmWidth = (error * gainP) + (motor->integral * gainI) + (derivative * gainD);
    // Preventing overloading the motor PWM width
    if(motor->pwmWidth > PWM_MAX_VALUE) {
        motor->pwmWidth = PWM_MAX_VALUE;
    }
}

// Call that will be made by the timer to correct the PID algorithm
void pidAdjustment()
{
    // Prevent adjustments to a stationary rover
    if((motor_controlData.leftMotor == &motor_controlData.stopMotorValues) || (motor_controlData.rightMotor == &motor_controlData.stopMotorValues)) {
        return;
    }
    // Perform correction to the PWM width values
    pidAlgorithm(motor_controlData.leftMotor, K_P, K_I, K_D);
    pidAlgorithm(motor_controlData.rightMotor, K_P, K_I, K_D);
    // Set the PWM width to the corrected value based on the PID control algorithm
    PLIB_OC_PulseWidth16BitSet(LEFT_OSCILLATOR, motor_controlData.leftMotor->pwmWidth);
    PLIB_OC_PulseWidth16BitSet(RIGHT_OSCILLATOR, motor_controlData.rightMotor->pwmWidth);
}

// Call that will made by the timer to gather sensor data
void updateSensorData()
{
    switch(motor_controlData.timer_states) {
        case enable_LED:
            setLEDState(pdTRUE);
            motor_controlData.timer_states = drive_IO_outputHigh;
            break;
        case drive_IO_outputHigh:
            setSensorPinsOutput();
            setSensorPinsHigh();
            motor_controlData.timer_states = drive_IO_input;
            break;
        case drive_IO_input:
            setSensorPinsInput();
            motor_controlData.timer_states = measure_disable_LED;
            break;
        case measure_disable_LED:
            readSensorPins();
            sendSensorData();
            setLEDState(pdFALSE);
            motor_controlData.timer_states = enable_LED;
            break;
    }
}

// Function called by mainapp to start using the sensor at the appropriate time
void startSensing()
{
    // Software timer for the sensor
    sensorTimerInitialize();
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
    /* External interrupt 1 corresponds to the right motor
       External interrupt 2 corresponds to the left motor */
    motor_controlData.motorControlQueue = xQueueCreate(16, sizeof(messageStructure));
    
    // Setting up default values that will be used throughout the system
    setInitialMotorValues(&motor_controlData.leftMotorMaxValues, TEST_START_VALUE, IDEAL_ENCODER_VALUE);
    setInitialMotorValues(&motor_controlData.rightMotorMaxValues, TEST_START_VALUE, IDEAL_ENCODER_VALUE);
    setInitialMotorValues(&motor_controlData.stopMotorValues, 0, 0);
    
    // Start the motor PWM widths at 0
    stopMotor();
    // Start the state machine at stop
    motor_controlData.states = stopMoving;
    // Start the state machine at enabling the LEDs
    motor_controlData.timer_states = enable_LED;
    // Defaulting the sensor data
    motor_controlData.sensorValues = 0;
    
    // Initializing the timer for rover movement correction by using the PID controller
    // Software timer for PID algorithm
    timerInitialize();
    
    // Initializing default values turning
    motor_controlData.turnCounter = 0;
    motor_controlData.turnStopValue = NINETY_DEGREES;
    
    // Setting the LED on
    //setLEDState(pdTRUE);
}


/******************************************************************************
  Function:
    void MOTOR_CONTROL_Tasks ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Tasks ( void )
{
    PLIB_OC_Enable(LEFT_OSCILLATOR);
    PLIB_OC_Enable(RIGHT_OSCILLATOR);
    // Timer period is 0.25 ms
    // Hardware (driver) timer
    DRV_TMR0_Start();
    // Need to activate timers 3 and 4 in order to read from the encoders at a frequency defined by me
    /*
     PLIB_TMR_Start(TMR_ID_3);
     PLIB_TMR_Start(TMR_ID_4);
     */
    
    makeLeftMotorForward(pdTRUE);
    makeRightMotorForward(pdTRUE);
    
    messageStructure tempMsg;
    uint16_t moving;
    
    // Temporary test
    //controlMotor(LEFT);
    
    while(1) {
        // Do encoder readings and powering motors
        if(xQueueReceive(motor_controlData.motorControlQueue, &tempMsg, portMAX_DELAY)) {
            // Processing
            // If the stop byte is set, stop everything
            if(tempMsg.messageContent[3]) {
                if(!((motor_controlData.leftMotor == &motor_controlData.stopMotorValues) || (motor_controlData.rightMotor == &motor_controlData.stopMotorValues))) {
                    stopMotor();
                    sendInternalCompleteMessage();
                }
                PLIB_OC_PulseWidth16BitSet(LEFT_OSCILLATOR, motor_controlData.leftMotor->pwmWidth);
                PLIB_OC_PulseWidth16BitSet(RIGHT_OSCILLATOR, motor_controlData.rightMotor->pwmWidth);
            }
            // Stop byte was not set, keep moving
            else {
                controlMotor(tempMsg.messageContent[0]);
                PLIB_OC_PulseWidth16BitSet(LEFT_OSCILLATOR, motor_controlData.leftMotor->pwmWidth);
                PLIB_OC_PulseWidth16BitSet(RIGHT_OSCILLATOR, motor_controlData.rightMotor->pwmWidth);
            }
        }
    }
}

 

/*******************************************************************************
 End of File
 */
