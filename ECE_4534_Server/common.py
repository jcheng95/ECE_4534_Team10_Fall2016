"""
Contains all the common constants and message structure
"""

# Frame Delimiters
START_BYTE  = b'\xB4'
END_BYTE    = b'\xC7'

# Sender Types
SERVER                = 0x01
PACMAN_CONTROL        = 0x02
GHOST_CONTROL         = 0x03
PACMAN_ROVER          = 0x04
GHOST_ROVER           = 0x05
SIMULATION            = 0x06

# Message Types
DEBUG                 = 0x01
PACMAN_COMMAND        = 0x02
GHOST_COMMAND         = 0x03
PACMAN_SENSOR         = 0x04
PACMAN_ROVER_COMPLETE = 0x05
GHOST_SENSOR          = 0x06
GHOST_ROVER_COMPLETE  = 0x07
INITIAL_ORDER         = 0x08

# Dictionaries for easy conversions
CHAR_TO_SENDER = {}
CHAR_TO_SENDER[SERVER]                          = "SERVER"
CHAR_TO_SENDER[PACMAN_CONTROL]                  = "Pac-Man Control PIC"
CHAR_TO_SENDER[GHOST_CONTROL]                   = "Ghost Control PIC"
CHAR_TO_SENDER[PACMAN_ROVER]                    = "Pac-Man Rover and Sensors PIC"
CHAR_TO_SENDER[GHOST_ROVER]                     = "Ghost Rover and Sensors PIC"
CHAR_TO_SENDER[SIMULATION]                      = "Simulation on Computer"

SENDER_TO_CHAR = {}
SENDER_TO_CHAR["SERVER"]                        = SERVER
SENDER_TO_CHAR["Pac-Man Control PIC"]           = PACMAN_CONTROL
SENDER_TO_CHAR["Ghost Control PIC"]             = GHOST_CONTROL
SENDER_TO_CHAR["Pac-Man Rover and Sensors PIC"] = PACMAN_ROVER
SENDER_TO_CHAR["Ghost rover and Sensors PIC"]   = GHOST_ROVER
SENDER_TO_CHAR["Simulation on Computer"]        = SIMULATION

CHAR_TO_MESSAGE = {}
CHAR_TO_MESSAGE[DEBUG]                          = "Debug Message"
CHAR_TO_MESSAGE[PACMAN_COMMAND]                 = "Pac-Man Movement Command"
CHAR_TO_MESSAGE[GHOST_COMMAND]                  = "Ghost Movement Command"
CHAR_TO_MESSAGE[PACMAN_SENSOR]                  = "Pac-Man Sensor Data"
CHAR_TO_MESSAGE[PACMAN_ROVER_COMPLETE]          = "Pac-Man Rover Movement Completed"
CHAR_TO_MESSAGE[GHOST_SENSOR]                   = "Ghost Sensor Data"
CHAR_TO_MESSAGE[GHOST_ROVER_COMPLETE]           = "Ghost Rover Movement Completed"
CHAR_TO_MESSAGE[INITIAL_ORDER]                  = "Order That Tokens Need To Be Picked Up"

class messageStructure:
    def __init__(self, sender, messageType, messageContent):
        self.sender = sender
        self.messageType = messageType
        self.messageContent = messageContent