#!/usr/bin/python3
"""Thruster control code for the maestro.
This is a port of Blake's 2020 Maestro code with minor changes from Ian for Pico, refacoted by Ian for Scion.
This code manually talks to the Pololu maestro driver, giving it instructions for how to set Scion's ESCs.
https://www.pololu.com/docs/0J40 link to maestro documentation
"""

import sys
import serial  # pip3 install pyserial
import struct
import time


class MaestroDriver:
    """Controls the Maestro.
    """
    def __init__(self, com_port: str, baud_rate=115200, lower_pulse_bound=1100, upper_pulse_bound=1900,
                 most_recent_thrusts=None):
        if most_recent_thrusts is None:
            most_recent_thrusts = [0, 0, 0, 0, 0, 0, 0, 0]
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.lower_pulse_bound = lower_pulse_bound
        self.upper_pulse_bound = upper_pulse_bound
        self.com_port = com_port
        self.usb = serial.Serial(self.com_port)
        self.most_recent_thrusts = most_recent_thrusts
        
        print('[MAESTRO] Initialized Class.')

    def set_thrusts(self, thrusts=None) -> None:
        """Sets thrusters.
        :param thrusts: Integer thruster values from -100 - 100.
        """
        print(f'[MAESTRO] Setting thrusts to {thrusts}')
        if thrusts is None:
            thrusts = [0, 0, 0, 0, 0, 0, 0, 0]
        for i in range(8):
            if thrusts[i] < -100 or thrusts[i] > 100 or len(thrusts) != 8:
                thrusts = self.most_recent_thrusts
        old_range = (100 + 100)
        new_range = (1900 - 1100)
        pulse_width = []
        # populating pulse width array
        for t in thrusts:
            if t < 0:  # Negative value check for clockwise motor spin
                t = t + 100
                pulse_width.append(abs((((t + 100) * new_range) / old_range) + 700))
            else:
                pulse_width.append((((t + 100) * new_range) / old_range) + 1100)

            pulse_width[-1] = round(pulse_width[-1] * 4)/4
        self.most_recent_thrusts = pulse_width

        # packing pulse width command
        for i in range(8):
            a = int(pulse_width[i] * 4)
            lower_bits = a & 0x7f
            upper_bits = (a >> 7) & 0x7f
            pulse_width_packed = struct.pack('>2h', lower_bits, upper_bits)
            message = bytearray([0x84, i, pulse_width_packed[1], pulse_width_packed[3]])
            try: 
                self.usb.write(message)
            except serial.serialutil.SerialException:
                time.sleep(0.02)
                self.usb = serial.Serial(self.com_port)


if __name__ == '__main__':
    print('Starting maestro driver demo...')
    device = ''
    if len(sys.argv) > 1:  # Did we get a device?
        device = sys.argv[1].replace(' ', '')
    else:
        print(f'Error: Expected argc > 1, number of args = {len(sys.argv)}. (Did you add the maestro device\'s COM '
              f'port?)')
        print('Exiting maestro driver demo...')
        sys.exit(1)
    maestro_driver = MaestroDriver(com_port=device)
    # Qualify for robosub
    sleep(10)  # Time for diver to unplug cable
    for i in range(2):
        maestro_driver.set_thrusts([30, 0, 30, 0, 30, 0, 30, 0])
        time.sleep(1)
        maestro_driver.set_thrusts([0, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.5)
    for i in range(10):
         maestro_driver.set_thrusts([0, 30, 0, 30, 0, 30, 0, 30])
        time.sleep(1.5)
        maestro_driver.set_thrusts([0, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.5)
    for i in range(5):
        maestro_driver.set_thrusts([-30, 0, -30, 0, -30, 0, -30, 0])
        time.sleep(1)
        maestro_driver.set_thrusts([0, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.5)
