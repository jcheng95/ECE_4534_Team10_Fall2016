"""
Contains all the common constants and message structure
"""

# Frame Delimiters
START_BYTE = b'\xB4'
END_BYTE = b'\xC7'

# Sender Types
SERVER   = 0x01
PIC      = 0x02
COMPUTER = 0x03

# Message Types
DEBUG       = 0x01
SENSOR      = 0x02
ACKNOWLEDGE = 0x03

# Dictionaries for easy conversions
CHAR_TO_SENDER = {}
CHAR_TO_SENDER[SERVER]      = "SERVER"
CHAR_TO_SENDER[PIC]         = "PIC"
CHAR_TO_SENDER[COMPUTER]    = "COMPUTER"

SENDER_TO_CHAR = {}
SENDER_TO_CHAR["SERVER"]    = SERVER
SENDER_TO_CHAR["PIC"]       = PIC
SENDER_TO_CHAR["COMPUTER"]  = COMPUTER

CHAR_TO_MESSAGE = {}
CHAR_TO_MESSAGE[DEBUG]       = "Debug Message"
CHAR_TO_MESSAGE[SENSOR]      = "Sensor Data"
CHAR_TO_MESSAGE[ACKNOWLEDGE] = "Acknowledgement from computer"

class messageStructure:
    def __init__(self, sender, messageType, messageContent):
        self.sender = sender
        self.messageType = messageType
        self.messageContent = messageContent