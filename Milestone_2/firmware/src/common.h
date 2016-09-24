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

/// NETWORK CONSTANTS ///

// All in bytes
#define MAX_MESSAGE_SIZE    8   // Total size of bytes for contents
#define MAX_BUFFER_SIZE     64  // Number of bytes allowed to be held from the buffer
#define MAX_TOTAL_SIZE      13  // Total size of content and delimiters and extraneous data

// SENDER TYPES //
#define SERVER      0x01
#define PIC         0x02
#define COMPUTER    0x03

// MESSAGE TYPES //
#define DEBUG       0x01
#define SENSOR      0x02
#define ACKNOWLEDGE 0x03

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
