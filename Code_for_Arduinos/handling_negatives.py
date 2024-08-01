import serial
import struct
import time

# Open serial port
arduino = serial.Serial(port = '/dev/cu.usbserial-110', baudrate = 9600, timeout = .1)

# Function to send and recieve arrays
def write_read(x):
    modified_array = [] # Initialize the modified array
    for number in x: # Send each number in array to the arduino and read the modified number back
        arduino.write(bytes(str(number) + '/n','utf-8'))
        #time.sleep(0.01)
        data = arduino.readline().strip().decode('utf-8')
        if data:
            modified_array.append(int(data))
    return modified_array

# Function to turn a string into an array
def extract_numbers(input_string):
    numbers = []  # Initialize an empty list to store numbers
    
    current_number = ''  # Initialize an empty string to store digits of the current number
    is_negative = False  # Flag to track if the current number is negative
    
    for char in input_string:
        if char.isdigit():
            current_number += char  # Append digit to the current number string
        elif char == '-' and not current_number:
            is_negative = True  # Set flag to indicate negative number
        elif current_number:
            if is_negative:
                current_number = '-' + current_number  # Prefix '-' to the current number string
                is_negative = False  # Reset the flag
            numbers.append(int(current_number))  # Convert the current number string to integer and add it to the list
            current_number = ''  # Reset the current number string
    
    if current_number:
        numbers.append(int(current_number))  # Add the last number if there's any
    
    return numbers


while True:

    numbers = input("Enter an array: ") # Get input array from user
    numbers_array = extract_numbers(numbers) # Convert string to list of integers

    # Send the array to arduino and receive the modified array
    modified_array = write_read(numbers_array);

    print("Modified Array:", modified_array)
