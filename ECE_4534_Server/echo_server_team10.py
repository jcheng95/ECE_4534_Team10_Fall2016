# In case this is used in a Linux-based OS
#!/usr/bin/env python

"""
Echo server used by the Raspberry Pi
"""

import socket, select
from queue import Queue
from threading import Thread
from common import *

# Array of clients
clientList = []
# Queue of socket messages to be sent out
messages = Queue()
# String for message
data = b''

# Counters for the number of messages sent and received
SEND_COUNT = 0
RECEIVE_COUNT = 0

# IP Data
HOST_IP = ''
PORT_NUMBER = 2000

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
    messagesize = message[4]
    msg = []
    # Takes the message as a bytes string
    for x in range (0, messagesize):
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
    sender = SERVER
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

# Converts a 16-bit integer from the ADC into centimeters
def convertSensorDataToCentimeters(value):
    sensorData = value & 0xFF

    return sensorData

# Parse socket packet for multiple messages
def separateMessages(packet):
    sender = 0
    count = 0
    type = 0
    size = 0
    msg = []
    for x in range(0, len(packet)):
        if packet[x] == START_BYTE:
            x += 1
            sender = packet[x]
            x += 1
            count = packet[x]
            x += 1
            type = packet[x]
            x += 1
            size = packet[x]
            x += 1
            for y in range(0, size):
                msg[y] = packet[x]
                x += 1
            if packet[x] == END_BYTE:
                messages.put(messageStructure(sender, type, msg))

# Converts the message contents of a sensor data message into readable content
def convertListToSensorData(message):
    val = (message[0] << 24) | (message[1] << 16) | (message[2] << 8) | message[3]
    return val

def listening():
    global clientList
    global data

    sensorData = 0

    # Socket parameters
    host = HOST_IP
    port = PORT_NUMBER
    backlog = 15
    size = 10

    # Socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Attempt to bind and listen to an IP address and port number
    s.bind((host,port))
    s.listen(backlog)
    clientList.append(s)
    while 1:
        # Get the list sockets which are ready to be read through select
        read_sockets,write_sockets,error_sockets = select.select(clientList,[],[])
        for sock in read_sockets:
            if sock == s:
                client, address = s.accept()

                # Checking if you connected to a new client
                clientList.append(client)
                print('New client at {}'.format(address))
            else:
                # Checking for a new message
                data = sock.recv(size)

                # Message exists
                if data:
                    # Ignore the WiFly "*HELLO*" message when the WiFly connects
                    if data == b'*HELLO*':
                        pass
                    else:
                        inMessage = convertFromMessage(data)

                        # Debug message
                        if inMessage.messageType == DEBUG:
                            # parse something here for display
                            print('Receive: {} ({}) - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                      inMessage.messageNumber,
                                                                      CHAR_TO_MESSAGE[inMessage.messageType],
                                                                      convertListToSensorData(inMessage.messageContent)))
                        # Command by Pac-Man Control PIC
                        elif inMessage.messageType == PACMAN_COMMAND:
                            # parse something here for display
                            print('Receive: {} ({}) - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                      inMessage.messageNumber,
                                                                      CHAR_TO_MESSAGE[inMessage.messageType],
                                                                      inMessage.messageContent))
                        # Command by Ghost Control PIC
                        elif inMessage.messageType == GHOST_COMMAND:
                            # parse something here for display
                            print('Receive: {} ({}) - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                      inMessage.messageNumber,
                                                                      CHAR_TO_MESSAGE[inMessage.messageType],
                                                                      inMessage.messageContent))
                        # Sensor data from Pac-Man Rover and Sensors PIC
                        elif inMessage.messageType == PACMAN_SENSOR:
                            # parse something here for display
                            print('Receive: {} ({}) - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                      inMessage.messageNumber,
                                                                      CHAR_TO_MESSAGE[inMessage.messageType],
                                                                      convertListToSensorData(inMessage.messageContent)))
                        # Movement Completion message from Pac-Man Rover and Sensors PIC
                        elif inMessage.messageType == PACMAN_ROVER_COMPLETE:
                            # parse something here for display
                            print('Receive: {} ({}) - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                      inMessage.messageNumber,
                                                                      CHAR_TO_MESSAGE[inMessage.messageType],
                                                                      convertListToSensorData(inMessage.messageContent)))
                        # Sensor data from Ghost Rover and Sensors PIC
                        elif inMessage.messageType == GHOST_SENSOR:
                            # parse something here for display
                            print('Receive: {} ({}) - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                      inMessage.messageNumber,
                                                                      CHAR_TO_MESSAGE[inMessage.messageType],
                                                                      convertListToSensorData(inMessage.messageContent)))
                        # Movement Completion message from Ghost Rover and Sensors PIC
                        elif inMessage.messageType == GHOST_ROVER_COMPLETE:
                            # parse something here for display
                            print('Receive: {} ({}) - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                      inMessage.messageNumber,
                                                                      CHAR_TO_MESSAGE[inMessage.messageType],
                                                                      inMessage.messageContent))
                        # Token order message from computer
                        elif inMessage.messageType == INITIAL_ORDER:
                            # parse something here for display
                            print('Receive: {} ({}) - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                      inMessage.messageNumber,
                                                                      CHAR_TO_MESSAGE[inMessage.messageType],
                                                                      inMessage.messageContent))

                        #Do not send the message to master socket and the client who has sent us the message
                        for sockets in clientList:
                            if sockets != s and sockets != sock :
                                try :
                                    sockets.send(data)
                                except :
                                    # broken socket connection may be, chat client pressed ctrl+c for example
                                    sockets.close()
                                    clientList.remove(sockets)
    for sockets in clientList:
        sockets.close()
        clientList.remove(sockets)

def main():
    global data
    global clientList

    while 1:
        """ Do Nothing """

if __name__ == '__main__':
    # Starting the listening thread
    listeningThread = Thread(target=listening, daemon=True)
    listeningThread.start()

    # Starting the main function
    main()