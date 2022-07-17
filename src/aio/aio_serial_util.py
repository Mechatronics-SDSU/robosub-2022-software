import serial
import time
import sys

# Hard coded port for serial connection

def write_read(x):
    arduino.write(x)
    time.sleep(0.05)
    data = arduino.readline()
    return data

if __name__ == '__main__':
    if len(sys.argv) > 1:  # Get USB mount location as an argument
        dev = sys.argv[1].replace(' ', '')

        arduino = serial.Serial(port=f'{dev}', baudrate=9600, timeout=.1)

        while True:
            command = input("Enter a command in bytes: ")
            command = int('0x' + command, 16).to_bytes(1, 'big')
            wr = b'i' + command + b'\n'
            print(wr)
            value = write_read(wr) # Hard coded message sent to AIO, currently set as "get kill status" 
            print(value)
    else:
        print('AIO Utils Error, sys.argv less than 2. (Did you add the device name?)')
        sys.exit(1)


