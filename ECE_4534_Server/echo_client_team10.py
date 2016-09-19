#!/usr/bin/env python

"""
A simple echo client
"""

import socket
from common import *

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

    # Client / Message sender
    sender = COMPUTER
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

def main():
    host = '192.168.71.135'
    port = 59784
    size = 1024

    sensorData = 0

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host,port))

    while 1:
        data = s.recv(size)

        # Data exists
        if data:
            inMessage = convertFromMessage(data)

            if inMessage.messageType == SENSOR:
                sensorData = convertToCentimeter(inMessage.messageContent)
                # Parse data -> send to visualization
                print('{}'.format(CHAR_TO_SENDER[inMessage.sender],
                                  CHAR_TO_MESSAGE[inMessage.messageType],
                                  sensorData))

                acknowledgeMessage = convertToMessage(ACKNOWLEDGE, inMessage.messageContent)
                s.send(acknowledgeMessage)
    s.close()

if __name__ == '__main__':
    main()