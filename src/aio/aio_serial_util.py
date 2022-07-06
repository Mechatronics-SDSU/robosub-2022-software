import serial
import time

# Hard coded port for serial connection
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)


def write_read(x):
    arduino.write(x)
    time.sleep(0.05)
    data = arduino.readline()
    return data


while True:
    command = input("Enter a command in bytes: ")
    value = write_read(b'i\x3F\n') # Hard coded message sent to AIO, currently set as "get kill status" 
    # value = write_read(b'i'+str(command).encode('utf-8')+b'\n')
    print(value)
