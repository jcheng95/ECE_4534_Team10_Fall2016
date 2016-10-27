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
    // Increment and limit message count
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
    newMessage.messageType = GHOST_SENSOR;
    newMessage.messageSize = 4;
    // MSB in index 0
    newMessage.messageContent[0] = val[0];
    newMessage.messageContent[1] = val[1];
    newMessage.messageContent[2] = val[2];
    newMessage.messageContent[3] = val[3];
    // Increment and limit message count
    ++mainappData.counter;
    mainappData.counter %= 256;
    // Send
    sendToTXQueue(newMessage);
}

void sendCommandMessage(unsigned char direction,unsigned int intersection,unsigned int heading)
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = mainappData.counter;
    newMessage.messageType = GHOST_COMMAND;
    newMessage.messageSize = 4;
    newMessage.messageContent[0] = direction;
    newMessage.messageContent[1] = intersection;
    newMessage.messageContent[2] = heading;
    newMessage.messageContent[3] = 0x00;
    // Increment and limit message count
    ++mainappData.counter;
    mainappData.counter %= 256;
    // Send
    sendToTXQueue(newMessage);
}

void sendGameOverMessage(void)
{
    messageStructure newMessage;
    newMessage.sender = MY_SENDER;
    newMessage.messageNumber = 0;
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

map initMap(){
    map theMap;
    //initial positions of pacman and ghost
    theMap.ghostPosition = 2;
    theMap.pacmanPosition = 5;
    theMap.ghostHeading = SOUTH;
    
    //initialize all intersection objects
    theMap.intersections[0].east = 1;
    theMap.intersections[0].south = 3;
    theMap.intersections[0].west = -1;
    theMap.intersections[0].north = -1;
    theMap.intersections[0].pacmanLocation = 0;
    
    theMap.intersections[1].east = 2;
    theMap.intersections[1].south = 4;
    theMap.intersections[1].west = 0;
    theMap.intersections[1].north = -1;
    theMap.intersections[1].pacmanLocation = 0;
    
    theMap.intersections[2].east = -1;
    theMap.intersections[2].south = 5;
    theMap.intersections[2].west = 1;
    theMap.intersections[2].north = -1;
    theMap.intersections[2].pacmanLocation = 0;
    
    theMap.intersections[3].east = 4;
    theMap.intersections[3].south = 8;
    theMap.intersections[3].west = -1;
    theMap.intersections[3].north = 0;
    theMap.intersections[3].pacmanLocation = 0;
    
    theMap.intersections[4].east = 5;
    theMap.intersections[4].south = -1;
    theMap.intersections[4].west = 3;
    theMap.intersections[4].north = 1;
    theMap.intersections[4].pacmanLocation = 0;
    
    theMap.intersections[5].east = -1;
    theMap.intersections[5].south = 7;
    theMap.intersections[5].west = 4;
    theMap.intersections[5].north = 2;
    theMap.intersections[5].pacmanLocation = 0;
    
    theMap.intersections[6].east = 7;
    theMap.intersections[6].south = -1;
    theMap.intersections[6].west = -1;
    theMap.intersections[6].north = -1;
    theMap.intersections[6].pacmanLocation = 0;
    
    theMap.intersections[7].east = -1;
    theMap.intersections[7].south = 12;
    theMap.intersections[7].west = 6;
    theMap.intersections[7].north = 5;
    theMap.intersections[7].pacmanLocation = 0;
    
    theMap.intersections[8].east = 9;
    theMap.intersections[8].south = 10;
    theMap.intersections[8].west = -1;
    theMap.intersections[8].north = 3;
    theMap.intersections[8].pacmanLocation = 0;
    
    theMap.intersections[9].east = -1;
    theMap.intersections[9].south = 11;
    theMap.intersections[9].west = 8;
    theMap.intersections[9].north = -1;
    theMap.intersections[9].pacmanLocation = 0;
    
    theMap.intersections[10].east = 11;
    theMap.intersections[10].south = -1;
    theMap.intersections[10].west = -1;
    theMap.intersections[10].north = 8;
    theMap.intersections[10].pacmanLocation = 0;
    
    theMap.intersections[11].east = 12;
    theMap.intersections[11].south = -1;
    theMap.intersections[11].west = 10;
    theMap.intersections[11].north = 9;
    theMap.intersections[11].pacmanLocation = 0;
    
    theMap.intersections[12].east = -1;
    theMap.intersections[12].south = -1;
    theMap.intersections[12].west = 11;
    theMap.intersections[12].north = 7;
    theMap.intersections[12].pacmanLocation = 0;
    
    theMap.intersections[theMap.pacmanPosition].pacmanLocation = 1;
    
    return theMap;
}

Direction pickDirection(map m){
    int tempPos = m.ghostPosition;
    //sendDebugMessage(10);
    while(m.intersections[tempPos].east > 0){
        if(m.intersections[m.intersections[tempPos].east].pacmanLocation){
            //sendDebugMessage(EAST);
            return EAST;
        }
        tempPos = m.intersections[tempPos].east;
    }
    
    //sendDebugMessage(20);
    tempPos = m.ghostPosition;
    
    while(m.intersections[tempPos].west > 0){
        //sendDebugMessage(m.intersections[m.intersections[tempPos].west].pacmanLocation);
        if(m.intersections[m.intersections[tempPos].west].pacmanLocation){
            //sendDebugMessage(WEST);
            return WEST;
        }
        tempPos = m.intersections[tempPos].west;
    }
    //sendDebugMessage(30);
    tempPos = m.ghostPosition;
    while(m.intersections[tempPos].north > 0){
        if(m.intersections[m.intersections[tempPos].north].pacmanLocation){
            //sendDebugMessage(NORTH);
            return NORTH;
        }
        tempPos = m.intersections[tempPos].north;
    }
    //sendDebugMessage(40);
    tempPos = m.ghostPosition;
    while(m.intersections[tempPos].south > 0){
        if(m.intersections[m.intersections[tempPos].south].pacmanLocation){
            //sendDebugMessage(SOUTH);
            return SOUTH;
        }
        tempPos = m.intersections[tempPos].south;
    }
    
    //pacman was not found, ghost should wander
    //pic a random direction for ghost
    //check if ghost can go forward first
    
    //sendDebugMessage(ERROR);
    if(m.ghostHeading == EAST && m.intersections[m.ghostPosition].east > 0){
        return EAST;
    }
    else if(m.ghostHeading == WEST && m.intersections[m.ghostPosition].west > 0){
        return WEST;
    }
    else if(m.ghostHeading == SOUTH && m.intersections[m.ghostPosition].south > 0){
        return SOUTH;
    }
    else if(m.ghostHeading == NORTH && m.intersections[m.ghostPosition].north > 0){
        return NORTH;
    }
    else {
        //sendDebugMessage(115);
        int d1 = rand() % 4;
        //pick a random direction since rover cant go forward
        while (1) {
            if (d1 < 1 && m.intersections[m.ghostPosition].south > 0) {
                return SOUTH;
            } else if (d1 < 2 && m.intersections[m.ghostPosition].west > 0) {
                return WEST;
            } else if (d1 < 3 && m.intersections[m.ghostPosition].east > 0) {
                return EAST;
            } else if (d1 < 4 && m.intersections[m.ghostPosition].north > 0) {
                return NORTH;
            } else {
                d1 = rand() % 4;
            }
        }
    }
    return ERROR;
}

void updateGhostPosition(map *m){
    if(m->ghostHeading == NORTH){
        m->ghostPosition = m->intersections[m->ghostPosition].north;
    }
    else if(m->ghostHeading == EAST){
        m->ghostPosition = m->intersections[m->ghostPosition].east;
    }
    else if(m->ghostHeading == WEST){
        m->ghostPosition = m->intersections[m->ghostPosition].west;
    }
    else if(m->ghostHeading == SOUTH){
        m->ghostPosition = m->intersections[m->ghostPosition].south;
    }
}

messageStructure pickAction(map *m, Direction dest){
    messageStructure msg;
    unsigned char direction;
    //build msg
    msg.sender = MY_SENDER;
    msg.messageNumber = mainappData.counter;
    msg.messageType = GHOST_COMMAND;
    msg.messageSize = 4;
    msg.messageContent[1] = m->ghostPosition;
    msg.messageContent[2] = m->ghostHeading;
    msg.messageContent[3] = 0x00;
    // Increment and limit message count
    ++mainappData.counter;
    mainappData.counter %= 256;
    
    switch(m->ghostHeading){
        case NORTH:
            if(dest == EAST){
                m->ghostHeading = EAST;
                direction = RIGHT;
            }
            else if(dest == SOUTH){
                m->ghostHeading = EAST;
                direction = RIGHT;
            }
            else if(dest == WEST){
                m->ghostHeading = WEST;
                direction = LEFT;
            }
            else{
                //sendDebugMessage(15);
                direction = FORWARD;
            }
            break;
            
        case SOUTH:
            if(dest == EAST){
                m->ghostHeading = EAST;
                direction = LEFT;
            }
            else if(dest == NORTH){
                m->ghostHeading = EAST;
                direction = LEFT;
            }
            else if(dest == WEST){
                m->ghostHeading = WEST;
                direction = RIGHT;
            }
            else{
                //sendDebugMessage(25);
                direction = FORWARD;
            }
            break;
            
        case EAST:
            if(dest == NORTH){
                m->ghostHeading = NORTH;
                direction = LEFT;
            }
            else if(dest == SOUTH){
                m->ghostHeading = SOUTH;
                direction = RIGHT;
            }
            else if(dest == WEST){
                m->ghostHeading = NORTH;
                direction = LEFT;
            }
            else{
                //sendDebugMessage(35);
                direction = FORWARD;
            }
            break;
            
        case WEST:
            if(dest == EAST){
                m->ghostHeading = NORTH;
                direction = RIGHT;
            }
            else if(dest == SOUTH){
                m->ghostHeading = SOUTH;
                direction = LEFT;
            }
            else if(dest == NORTH){
                m->ghostHeading = NORTH;
                direction = RIGHT;
            }
            else{
                //sendDebugMessage(45);
                direction = FORWARD;
            }
            break;
            
    }
    if(direction == FORWARD){
        updateGhostPosition(m);
    }
    msg.messageContent[0] = direction;
    return msg;
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
    bool turning = 1;
    //init msg structure and UART
    messageStructure tempMsg;

    PLIB_USART_Enable(USART_ID_1);
    //init map and Intersections
    map theMap;
    theMap = initMap();
    
    while(1) {
        if(xQueueReceive(mainappData.mainQueue, &tempMsg, portMAX_DELAY)) {
            // Filter messages
            if(tempMsg.messageType == INITIAL_ORDER) {
                // Filter out content
                sendCommandMessage(FORWARD,theMap.ghostPosition,theMap.ghostHeading);
                updateGhostPosition(&theMap);
                //sendDebugMessage(20);
            }
            else if(tempMsg.messageType == GHOST_SENSOR) {
                // Filter out content
                sendDebugMessage(90);
            }
            else if(tempMsg.messageType == PACMAN_POSITION) {
                //update pacman position
            }
            else if(tempMsg.messageType == GHOST_ROVER_COMPLETE) {
                turning = 1;
                if(theMap.ghostPosition == theMap.pacmanPosition){
                    sendDebugMessage(theMap.ghostPosition);
                    sendGameOverMessage();
                }
                //sendDebugMessage(10);
                // Filter out content
                Direction dir = pickDirection(theMap);
                messageStructure instruction = pickAction(&theMap, dir);
                sendToTXQueue(instruction);
                if(instruction.messageContent[0] == FORWARD){
                    turning = 0;
                }
                
                //sendDebugMessage(100);
                while(turning){

                    if(xQueueReceive(mainappData.mainQueue, &tempMsg, portMAX_DELAY)){

                        if(tempMsg.messageType == GHOST_ROVER_COMPLETE){
                            messageStructure instruction = pickAction(&theMap, dir);
                            sendToTXQueue(instruction);
                            if(instruction.messageContent[0] == FORWARD){
                                turning = 0;
                            }
                            //sendDebugMessage(theMap.ghostHeading);
                        }
                    }
                }
                //sendDebugMessage(100);
//                if(xQueueReceive(mainappData.mainQueue, &tempMsg, portMAX_DELAY)){
//                    if(tempMsg.messageType == GHOST_ROVER_COMPLETE){
//                        //sendDebugMessage(60);
//                        sendCommandMessage(FORWARD,theMap.ghostPosition,theMap.ghostHeading);
//                        updateGhostPosition(&theMap);
//                    }
//                }
                //sendDebugMessage(70);
            }
            else if(tempMsg.messageType == DEBUG) {
                // Filter out content
                sendDebugMessage(40);
            }
            else if(tempMsg.messageType == GAME_OVER) {
                // Filter out content
                sendGameOverMessage();
            }
        }
    }
}

 

/*******************************************************************************
 End of File
 */
