"""Depth sensor driver.
Handles automatic USB discovery similar to the kill switch.
Running as main will run some test code to validate connection, importing will allow a generic sensor driver to
instantiate the class defined here.
"""

import serial.tools.list_ports
import time
import sys


class Depth:
    """Handles all functionality related to serial communication with the depth sensor.
    """
    def __init__(self) -> None:
        self.device_name = ''
        self.device = None
        self.set_device()
        self.com_test()

    def set_device(self) -> None:
        """Grab the device by using serial tools to search for the Depth Sensor.
        """
        for port in serial.tools.list_ports.comports():
            with serial.Serial(port.device, 9600) as dev:
                if 'Depth Sensor' in str(dev.readline()):  # Line 13, depth_sensor.ino
                    self.device_name = port.device

    def com_test(self) -> None:
        """Validate we can connect over serial at the specified baud rate.
        """
        try:
            self.device = serial.Serial(self.device_name, 9600)
            time.sleep(1)
        except serial.serialutil.SerialException as e:
            print("Error: Failed to locate depth sensor")
            print(e)
            sys.exit(1)
        except ValueError as e:
            print("Error: Baud Rate error")
            print(e)
            sys.exit(1)

    def get_state(self) -> list:
        """Read the current state of the sensor. Returns as list.
        """
        if self.device is not None:
            try:
                self.device.write(b'g\n')  # g\n = get state
                resp = self.device.readline()
                if resp[0] == ord('r'):
                    ret = []
                    i = 1
                    while resp[i] != '\n' or resp[i] != ord('\n'):
                        ret.append(resp[i])
                    return ret
            except serial.SerialException as e:
                print("Error: Serial communication error when attempting to get state, attempting to re-establish...")
                self.com_test()
                self.get_state()

    def close(self) -> None:
        """Close connection to the device properly.
        """
        if self.device is not None:
            try:
                self.device.close()
            except AttributeError as e:
                print("Error: Failed to close depth sensor")
                print(e)
                sys.exit(1)


if __name__ == "__main__":
    print("Testing Depth Sensor.")
    DP = Depth()
    while True:
        print(f'Depth Sensor Test Reading: {DP.get_state()}')
        time.sleep(0.01)
else:
    print('Imported Depth Sensor module.')
