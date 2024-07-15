# Importing Libraries

import serial
import time

arduino = serial.Serial(port = '/dev/cu.usbserial-1130', baudrate = 9600, timeout=.1)

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.1)
    data = arduino.readline()
    return data.decode('utf-8')

while True:
    num = input("Enter a number or letter: ") # Taking input from user
    value = write_read(num)
    print(value) 
