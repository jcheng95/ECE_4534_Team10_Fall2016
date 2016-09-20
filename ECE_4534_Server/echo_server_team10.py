# In case this is used in a Linux-based OS
#!/usr/bin/env python

"""
Echo server used by the Raspberry Pi
"""

import socket, select
import time
from threading import Thread
from common import *

# Array of clients
clientList = []
# String for message
data = b''

# Counters for the number of messages sent and received
SEND_COUNT = 0
RECEIVE_COUNT = 0

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

    # Return the full message
    return START_BYTE + incompleteMessage + END_BYTE

def listening():
    global clientList
    global data

    sensorData = 0

    # Socket parameters
    host = ''
    port = 2000
    backlog = 10
    size = 1024

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
                    if data == b'*HELLO*':
                        pass
                    else:
                        inMessage = convertFromMessage(data)

                        # Debug message
                        if inMessage.messageType == DEBUG:
                            # parse something here for display
                            print('Receive: {} - {} : {}'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                 CHAR_TO_MESSAGE[inMessage.messageType],
                                                                 sensorData))
                        # Sensor data from PIC
                        elif inMessage.messageType == SENSOR:
                            # parse something here for display
                            print('Receive: {} - {} : {} cm'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                 CHAR_TO_MESSAGE[inMessage.messageType],
                                                                 sensorData))
                        # Sensor data acknowledge from a computer
                        elif inMessage.messageType == ACKNOWLEDGE:
                            # parse something here for display
                            print('Receive: {} - {} : {} cm'.format(CHAR_TO_SENDER[inMessage.sender],
                                                                 CHAR_TO_MESSAGE[inMessage.messageType],
                                                                 sensorData))

                        #Do not send the message to master socket and the client who has sent us the message
                        for sockets in clientList:
                            if sockets != s and sockets != sock :
                                try :
                                    sockets.send(data)
                                except :
                                    # broken socket connection may be, chat client pressed ctrl+c for example
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