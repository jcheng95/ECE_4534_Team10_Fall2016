/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    motor_control.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include <timers.h>
#include <queue.h>
#include "common.h"
#include "mainapp_public.h"

/*
Motor Pins:
    Right Motor (J3 - S(A/B)1):
    3  - OC1/PWM
    78 - Direction
    37 - Encoder1 (Max32 = TMR3/Pin 22) (SB2-In) <- This is the ideal encoder value?
    7  - Encoder2 INT2 (SA2-In)                  <- This is the real encoder value?
 
    Left Motor (J6 - S(A/B)2):
    5  - OC2/PWM
    4  - Direction
    A6 - Encoder1 (Max32 = TMR4/Pin 23) (SB1-In) <- This is the ideal encoder value?
    2  - Encoder2 INT1 (SA1-In)                  <- This is the real encoder value?
 */

/*
 Motor gear ratio - 298 : 1
 Exact gear ratio - 297.92 : 1  ==  (25 * 34 * 37 * 35 * 38) / (12 * 9 * 10 * 13 * 10)
 
 There are 12 encoder ticks for every revolution of the smaller gear. So one full rotation
 of the larger gear would result in (12 * 297.92) = 3575.04 or, by applying the approximation,
 (12 * 298) = 3576
 */

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef enum {
    movingForward = 0x01,
    movingBackward = 0x02,
    movingLeft = 0x03,
    movingRight = 0x04,
    stopMoving = 0x05
} CURRENT_DIRECTION;   

typedef enum {
    enable_LED = 0x00,
    drive_IO_outputHigh = 0x01,
    drive_IO_input = 0x02,
    measure_disable_LED = 0x03
} TIMER_FSM;

// Struct to represent a motor with particular values
typedef struct {
    int16_t pwmWidth;
    uint16_t encoderValue;
    uint16_t previousEncoderValue;
    uint16_t idealEncoderValue;
    uint16_t integral;
    uint16_t previousError;
} Motor;
    
typedef struct
{
    // Queue for commands for movement
    QueueHandle_t motorControlQueue;
    TimerHandle_t motorAdjustmentTimer;
    
    // Turn counter and stop values
    unsigned int turnCounter;
    unsigned int turnStopValue;
    
    /*
       Data type used to contain the booleans of each sensor 
       Each sensor represents a bit in the data type where
       the MSB is the highest number sensor in the list
    */
    unsigned char sensorValues;
    
    // Motor values used to assign to the left and right motors for easy manipulation of the speed and direction of the rover
    Motor leftMotorMaxValues;
    Motor rightMotorMaxValues;
    Motor stopMotorValues;
    
    // Actual motors
    Motor *leftMotor;
    Motor *rightMotor;
    
    // State machine for the current direction for controlling the turns
    CURRENT_DIRECTION states;
    // State machine for the timer for sequence
    TIMER_FSM timer_states;
} MOTOR_CONTROL_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MOTOR_CONTROL_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MOTOR_CONTROL_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void MOTOR_CONTROL_Initialize ( void );


/*******************************************************************************
  Function:
    void MOTOR_CONTROL_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MOTOR_CONTROL_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void MOTOR_CONTROL_Tasks( void );


#endif /* _MOTOR_CONTROL_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

