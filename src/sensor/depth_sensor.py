#!/usr/bin/python3
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
    def __init__(self, dev_name: str, cal_offset=0.0 ) -> None:
        self.device_name = dev_name
        self.device = dev_name
        #self.set_device()
        self.com_test()
        self.cal_offset = cal_offset

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
            str(self.device.readline())
            time.sleep(1)
        except serial.serialutil.SerialException as e:
            print("Error: Failed to locate depth sensor")
            print(e)
            sys.exit(1)
        except ValueError as e:
            print("Error: Baud Rate error")
            print(e)
            sys.exit(1)

    def get_state(self) -> float:
        """Read the current state of the sensor. Returns as list.
        """
        if self.device is not None:
            try:
                self.device.write(b'g\n')  # g\n = get state
                resp = self.device.readline()
                #print(f"resp: {resp}")
                if "None" in str(resp):
                    return 0.0
                if resp[0] == ord('r'):
                    return float(resp[1:-1])+self.cal_offset
            except serial.SerialException as e:
                print("Error: Serial communication error when attempting to get state, attempting to re-establish...")
                # self.com_test()
                # self.get_state()
        else:
            return 0.0

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

def main(com_port: str) -> None:
    """Test Driver for basic Depth functionality.
    """
    print("Testing Depth Sensor.")
    DP = Depth(dev_name=com_port)
    while True:
        s = DP.get_state()
        print(f'Depth Sensor Test Reading: {s} {type(s)}')
        time.sleep(0.01)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        depth_dev_port = sys.argv[1].replace(' ', '')
        main(com_port=depth_dev_port)
    else:
        print("Error: not enough arguments. (Was the depth dev port passed?)")
        sys.exit(1)