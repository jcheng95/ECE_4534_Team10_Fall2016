/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#include "system_config.h"
#include "system_definitions.h"
#include "mainapp.h"
#include "uart_tx.h"
#include "uart_rx.h"
#include "adc_app.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************


 
static void _SYS_Tasks ( void );
static void _MAINAPP_Tasks(void);
static void _UART_TX_Tasks(void);
static void _UART_RX_Tasks(void);
static void _ADC_APP_Tasks(void);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

void SYS_Tasks ( void )
{
    /* Create OS Thread for Sys Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_Tasks,
                "Sys Tasks",
                1024, NULL, 0, NULL);

    /* Create OS Thread for MAINAPP Tasks. */
    xTaskCreate((TaskFunction_t) _MAINAPP_Tasks,
                "MAINAPP Tasks",
                4096, NULL, 1, NULL);

    /* Create OS Thread for UART_TX Tasks. */
    xTaskCreate((TaskFunction_t) _UART_TX_Tasks,
                "UART_TX Tasks",
                4096, NULL, 1, NULL);

    /* Create OS Thread for UART_RX Tasks. */
    xTaskCreate((TaskFunction_t) _UART_RX_Tasks,
                "UART_RX Tasks",
                4096, NULL, 1, NULL);

    /* Create OS Thread for ADC_APP Tasks. */
    xTaskCreate((TaskFunction_t) _ADC_APP_Tasks,
                "ADC_APP Tasks",
                1024, NULL, 1, NULL);

    /**************
     * Start RTOS * 
     **************/
    vTaskStartScheduler(); /* This function never returns. */
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( void )

  Summary:
    Maintains state machines of system modules.
*/
static void _SYS_Tasks ( void)
{
    while(1)
    {
        /* Maintain system services */
        SYS_DEVCON_Tasks(sysObj.sysDevcon);

        /* Maintain Device Drivers */

        /* Maintain Middleware */

        /* Task Delay */
    }
}


/*******************************************************************************
  Function:
    void _MAINAPP_Tasks ( void )

  Summary:
    Maintains state machine of MAINAPP.
*/

static void _MAINAPP_Tasks(void)
{
    while(1)
    {
        MAINAPP_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _UART_TX_Tasks ( void )

  Summary:
    Maintains state machine of UART_TX.
*/

static void _UART_TX_Tasks(void)
{
    while(1)
    {
        UART_TX_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _UART_RX_Tasks ( void )

  Summary:
    Maintains state machine of UART_RX.
*/

static void _UART_RX_Tasks(void)
{
    while(1)
    {
        UART_RX_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _ADC_APP_Tasks ( void )

  Summary:
    Maintains state machine of ADC_APP.
*/

static void _ADC_APP_Tasks(void)
{
    while(1)
    {
        ADC_APP_Tasks();
    }
}


/*******************************************************************************
 End of File
 */

