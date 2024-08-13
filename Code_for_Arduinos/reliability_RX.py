import serial
serial_success = False
# Open the serial port
while not serial_success:
    try:
        ser = serial.Serial('COM4', 9600)  # Replace 'COM1' with the appropriate port and baud rate
        serial_success = True
    except serial.SerialException:
        serial_success = False
# Open the file in write mode
print("Serial port opened successfully")
file = open('receiving.txt', 'w')
 
# Initialize count
count = 0
# Read and save the serial data
while count < 1000:
    if ser.in_waiting > 0:
        data = ser.readline().decode().strip()
        file.write(data + '\n')
        # print(data)  # Optional: Print the data to the console
        file.flush()  # Optional: Flush the file buffer
        count += 1
# Close the file and serial port
file.close()
ser.close()
