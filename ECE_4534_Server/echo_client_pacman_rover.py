#!/usr/bin/env python

"""
An echo client for the ECE 4534 Team 10 project for Fall 2016.
"""

import socket
import atexit
from common import *

# Message Counts
SEND_COUNT = 0
RECEIVE_COUNT = 0

# IP Data
HOST_IP = '10.0.0.1'
PORT_NUMBER = 2000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Close all sockets still listening to the host
def closeLooseConnections():
    s.close()

# Register closeLooseConnections() as a function to be called upon exiting
atexit.register(closeLooseConnections)

# Converts an array of bytes to a messageStructure object that can be used to send out a message
# Really this is just a fancy way of extracting data I want to keep a hold of in a class
def convertFromMessage(message):
    global RECEIVE_COUNT

    # Increment the send counter
    RECEIVE_COUNT += 1
    RECEIVE_COUNT = RECEIVE_COUNT % 256
    # The sender / client of the message
    client = message[1]
    # The message number for the number of messages this client has received
    messageNumber = message[2]
    # Message type
    messageType = message[3]
    # Message size <- only used for double-checking a message wasn't lost
    messageSize = message[4]
    msg = []
    # Takes the message as a bytes string
    for x in range (0, messageSize):
        try:
            msg.append(message[x + 5])
        except:
            pass

    # Creates an internal message for easy data extraction
    convertedMessage = messageStructure(client, messageNumber, messageType, msg)
    return convertedMessage

# Converts to a message that can be sent wirelessly
def convertToMessage(messageType, message):
    global SEND_COUNT

    # Client / Message sender
    sender = SIMULATION
    # The message number for the number of messages this client has sent
    messageNo = SEND_COUNT

    # Message type
    type = messageType
    # Message size
    msgSize = len(message)
    # Message without the start and end bytes
    incompleteMessage = bytes([sender, messageNo, type, msgSize]) + message

    # Increment the send counter
    SEND_COUNT += 1
    SEND_COUNT = SEND_COUNT % 256

    # Return the full message
    return START_BYTE + incompleteMessage + END_BYTE

# Converts to a message (only contents) that is for a debug message
def toDebugMessage(value):
    """ MSB is the first byte """
    byte4 = (int(value) & 0xFF000000) >> 24
    byte3 = (int(value) & 0x00FF0000) >> 16
    byte2 = (int(value) & 0x0000FF00) >> 8
    byte1 = (int(value) & 0x000000FF)
    return bytes([byte4, byte3, byte2, byte1])

# Converts to a message (only contents) that is for a command message
def toCommandMessage(direction, xPos, yPos):
    return bytes([direction, xPos, yPos, 0])

# Converts the message contents of a sensor data message into readable content
def convertListToSensorData(message):
    val = (message[0] << 24) | (message[1] << 16) | (message[2] << 8) | message[3]
    return val

# Converts a 16-bit integer from the ADC into centimeters
def convertSensorDataToCentimeters(value):
    sensorData = value & 0xFF

    return sensorData

# Converts the order of tokens into individual digits then to bytes
def byteify(numbers):
    first = int(numbers[0])
    second = int(numbers[1])
    third = int(numbers[2])
    fourth = int(numbers[3])

    return first, second, third, fourth

# Sends the token order message
def sendInitialMessage(value, sock):
    first, second, third, fourth = byteify(value)
    initialMessage = convertToMessage(INITIAL_ORDER, bytes([first, second, third, fourth]))
    sock.send(initialMessage)

# Sends a rover movement message
def sendRoverMoveMessage(sock, direction, expectedX, expectedY, stop):
    roverMoveMessage = convertToMessage(PACMAN_COMMAND, bytes([direction, expectedX, expectedY, stop]))
    sock.send(roverMoveMessage)

def main():
    global s

    host = HOST_IP
    port = PORT_NUMBER
    size = 1024

    # Poll for user input
    #order = input('Please enter the order (e.g. 1234): ')

    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host,port))

    # Once a host is connected to, send the message
    #sendInitialMessage(order, s)

    while 1:
        direct = input('Please enter a direction (w, a, s, d, or j): ')
        if direct == 'w':
            sendRoverMoveMessage(s, FORWARD, 0, 0, 0)
        elif direct == 's':
            sendRoverMoveMessage(s, BACKWARD, 0, 0, 0)
        elif direct == 'a':
            sendRoverMoveMessage(s, LEFT, 0, 0, 0)
        elif direct == 'd':
            sendRoverMoveMessage(s, RIGHT, 0, 0, 0)
        elif direct == 'j':
            sendRoverMoveMessage(s, FORWARD, 0, 0, 1)

if __name__ == '__main__':
    main()