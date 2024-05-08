import socket
import numpy as np
import math

HOST = '169.254.186.240'
PORT = 5000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Initialized socket')
s.connect((HOST,PORT))
print('connected to host at port ', PORT)

# loop while waiting for input
flag = 'Continue'
while True:
    messageSize = int.from_bytes(s.recv(1), byteorder='little')
    byteMessage = s.recv(messageSize)
    message = np.ones(messageSize)
    for i in range(messageSize):
        message[i] = byteMessage[i]
    
    # print("Message Size:", messageSize)
    # print("Message:", message)
    
    reply = np.ones(shape = messageSize, dtype=np.uint8)
    for i in range(messageSize):
        # This section results in a list of 'uint8' values
        reply[i] = math.floor(message[i])
    s.send(reply)
    # print("Reply sent:", reply)
    
    if flag == 'Terminate' or messageSize == 255:
        break