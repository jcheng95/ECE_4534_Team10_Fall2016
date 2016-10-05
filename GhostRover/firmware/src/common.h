/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _COMMON_H    /* Guard against multiple inclusion */
#define _COMMON_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/// NETWORK CONSTANTS ///

// All in bytes
#define MAX_MESSAGE_SIZE    8   // Total size of bytes for contents
#define MAX_BUFFER_SIZE     64  // Number of bytes allowed to be held from the buffer
#define MAX_TOTAL_SIZE      10  // Total size of content and delimiters and extraneous data
#define MAX_USABLE_SIZE     15  // Absolute maximum size of content and delimiters and extraneous data

// SENDER TYPES //
#define SERVER              0x01
#define PACMAN_CONTROL      0x02
#define GHOST_CONTROL       0x03
#define PACMAN_ROVER        0x04
#define GHOST_ROVER         0x05
#define SIMULATION          0x06
    
/// USER CONSTANT ///
#define MY_SENDER   GHOST_ROVER

// MESSAGE TYPES //
#define DEBUG                   0x01
#define PACMAN_COMMAND          0x02
#define GHOST_COMMAND           0x03
#define PACMAN_SENSOR           0x04
#define PACMAN_ROVER_COMPLETE   0x05
#define GHOST_SENSOR            0x06
#define GHOST_ROVER_COMPLETE    0x07
#define INITIAL_ORDER           0x08
#define GAME_OVER               0x0A
    
/// Directions ///
#define FORWARD     0x01
#define BACKWARD    0x02
#define LEFT        0x03
#define RIGHT       0x04

// FRAME DELIMITERS //
#define START_BYTE  ((unsigned char) 0xB4)
#define END_BYTE    ((unsigned char) 0xC7)

/// OTHER CONSTANTS ///
#define MAX_SAMPLE_SIZE 16

typedef struct {
    unsigned char sender;
    unsigned char messageNumber;
    unsigned char messageType;
    unsigned char messageSize;
    unsigned char messageContent[MAX_MESSAGE_SIZE];
} messageStructure;

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _COMMON_H */

/* *****************************************************************************
 End of File
 */
