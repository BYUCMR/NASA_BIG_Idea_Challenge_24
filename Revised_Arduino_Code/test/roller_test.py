import serial
import time

def test_motors(*messages, num_times=5, sleep_time=5):
    ser = serial.Serial(port="COM5", baudrate=115200)

    if ser.is_open:
        print(f"Connected to {ser.name}")
    else:
        print(f"Could not connect to {ser.name}")
        return
    
    if sleep_time < 2:
        sleep_time = 2
    
    time.sleep(2)
    
    for _ in range(num_times):
        for j in range(len(messages)):
            print(f"Sending: {messages[j]}")
            ser.write(f"{messages[j]}".encode())
                    
            time.sleep(sleep_time)

    ser.close()

def create_strlist(int_list):
    str_list = list(map(str, int_list))
    return ",".join(str_list)

    
if __name__ == "__main__":
    import numpy as np
    # int_list = np.array([0, 1, 0, 0, 1, 1, 1, 0])*-1000
    int_list = np.array([0, 0, 0, 0, 1, 1, 1, 0])*-1000
    
    input1_str = create_strlist(int_list)

    test_motors(input1_str, num_times=1, sleep_time=1)