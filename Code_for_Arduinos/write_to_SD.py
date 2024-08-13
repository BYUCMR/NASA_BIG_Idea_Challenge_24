import serial
import time
# Open the serial port
ser = serial.Serial('COM4', 9600)  # Replace 'COM1' with the appropriate port and baud rate

# Open the file in write mode
file = open('output.txt', 'w')
current_time = time.time()

# Read and save the serial data
while time.time() - current_time < 10:
    if ser.in_waiting > 0:
        data = ser.readline().decode().strip()
        file.write(data + '\n')
        print(data)  # Optional: Print the data to the console
        file.flush()  # Optional: Flush the file buffer

# Close the file and serial port
file.close()
ser.close()