#!/usr/bin/env python

"""
An echo client for the ECE 4534 Team 10 project for Fall 2016.
"""

import socket
from common import *

# Message Counts
SEND_COUNT = 0
RECEIVE_COUNT = 0

# IP Data
HOST_IP = '10.0.0.1'
PORT_NUMBER = 2000

# Converts an array of bytes to a messageStructure object that can be used to send out a message
# Really this is just a fancy way of extracting data I want to keep a hold of in a class
def convertFromMessage(message):
    global RECEIVE_COUNT

    # Increment the send counter
    RECEIVE_COUNT += 1
    # The sender / client of the message
    client = message[1]
    # The message number for the number of messages this client has received
    RECEIVE_COUNT = message[2]
    # Message type
    messageType = message[3]
    # Message size <- only used for double-checking a message wasn't lost
    messagesize = message[4]
    msg = []
    # Takes the message as a bytes string
    for x in range (0, messagesize):
        try:
            msg.append(message[x + 5])
        except:
            pass

    # Creates an internal message for easy data extraction
    convertedMessage = messageStructure(client, messageType, msg)
    return convertedMessage

# Converts to a message that can be sent wirelessly
def convertToMessage(messageType, message):
    global SEND_COUNT

    SEND_COUNT %= 256

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

    # Return the full message
    return START_BYTE + incompleteMessage + END_BYTE

# Converts the message contents of a sensor data message into readable content
def convertToCentimeter(messageContent):
    convertedVal = messageContent[0]

    return convertedVal

# Converts the order of tokens into individual digits then to bytes
def byteify(numbers):
    temp = numbers
    fourth = temp % 10
    temp /= 10

    third = temp % 10
    temp /= 10

    second = temp % 10
    temp /= 10

    first = temp % 10
    temp /= 10

    return first, second, third, fourth

# Sends the token order message
def sendInitialMessage(value, sock):
    first, second, third, fourth = byteify(value)
    initialMessage = convertToMessage(INITIAL_ORDER, bytes([first, second, third, fourth]))
    sock.send(initialMessage)

def main():
    host = HOST_IP
    port = PORT_NUMBER
    size = 1024

    # Poll for user input
    order = input('Please enter the order (e.g. 1234): ')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host,port))

    # Once a host is connected to, send the message
    sendInitialMessage(order, s)

    while 1:
        data = s.recv(size)

        # Data exists
        if data:
            inMessage = convertFromMessage(data)

            # Debug message
            if inMessage.messageType == DEBUG:
                # parse something here for display
                print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                     CHAR_TO_MESSAGE[inMessage.messageType],
                                                     inMessage.messageContent))
            # Command by Pac-Man Cntrol PIC
            elif inMessage.messageType == PACMAN_COMMAND:
                # parse something here for display
                print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                     CHAR_TO_MESSAGE[inMessage.messageType],
                                                     inMessage.messageContent))
            # Sensor data acknowledge from a computer
            elif inMessage.messageType == GHOST_COMMAND:
                # parse something here for display
                print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                     CHAR_TO_MESSAGE[inMessage.messageType],
                                                     inMessage.messageContent))
            elif inMessage.messageType == PACMAN_SENSOR:
                # parse something here for display
                print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                     CHAR_TO_MESSAGE[inMessage.messageType],
                                                     inMessage.messageContent))
            elif inMessage.messageType == PACMAN_ROVER_COMPLETE:
                # parse something here for display
                print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                     CHAR_TO_MESSAGE[inMessage.messageType],
                                                     inMessage.messageContent))
            elif inMessage.messageType == GHOST_SENSOR:
                # parse something here for display
                print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                     CHAR_TO_MESSAGE[inMessage.messageType],
                                                     inMessage.messageContent))
            elif inMessage.messageType == GHOST_ROVER_COMPLETE:
                # parse something here for display
                print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                     CHAR_TO_MESSAGE[inMessage.messageType],
                                                     inMessage.messageContent))
            elif inMessage.messageType == INITIAL_ORDER:
                # parse something here for display
                print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                     CHAR_TO_MESSAGE[inMessage.messageType],
                                                     inMessage.messageContent))
    s.close()

if __name__ == '__main__':
    main()