"""
Leak detector gets state of leak detection as boolean value from arduino. 
Outputs leak state; either a leak has or has not been detected. 
"""
import serial

port = 'COM4'

def sensorOutput(port):
    arduino = serial.Serial(port, baudrate=9600)
    leakSensor = arduino.readline()

    if leakSensor is True:
        print("Leak detected!")
    elif leakSensor is False:
        print("No leaks currently detected")

if __name__ == '__main__':
    sensorOutput("COM4")