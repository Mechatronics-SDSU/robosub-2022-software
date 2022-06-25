import serial
import time

arduino = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=.1)


def write_read(x):
    arduino.write(x)
    time.sleep(0.05)
    data = arduino.readline()
    return data


while True:
    command = input("Enter a command in bytes: ")
    value = write_read(b'i\x3F\n')
    # value = write_read(b'i'+str(command).encode('utf-8')+b'\n')
    print(value)
