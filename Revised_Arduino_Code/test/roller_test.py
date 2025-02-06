import serial
import time

def test_motors(*messages, num_times=5, sleep_time=5):
    ser = serial.Serial(port="COM4", baudrate=115200)

    if ser.is_open:
        print(f"Connected to {ser.name}")
    else:
        print(f"Could not connect to {ser.name}")
        return
    
    if sleep_time < 2:
        sleep_time = 2
    
    time.sleep(5)
    
    for _ in range(num_times):
        for j in range(len(messages)):
            print(f"Sending: {messages[j]}")
            ser.write(f"{messages[j]}".encode())
            
            time.sleep(sleep_time)

    ser.close()

    
if __name__ == "__main__":
    input1 = ["0"]*8
    input2 = ["0"]*8
    input2[3] = "-500"

    input1_str = ",".join(input1)
    input2_str = ",".join(input2)
    test_motors(input1_str, input2_str, num_times=10, sleep_time=5)