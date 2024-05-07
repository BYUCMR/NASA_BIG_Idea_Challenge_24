import socket

HOST = '169.254.186.240'
PORT = 5000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Initialized socket')
s.connect((HOST,PORT))
print('connected to host at port ', PORT)

# loop while waiting for input

while True:
    command = input('Enter your command:')
    s.send(command.encode('utf-8'))
    reply = s.recv(5)
    print(reply)
    if reply == 'Terminate':
        break