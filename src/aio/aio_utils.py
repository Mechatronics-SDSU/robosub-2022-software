#!/usr/bin/env python3
"""AIO class wrapper for interfacing with AIO.

This is meant to be imported and used by a ROS driver:
ROS driver will instantiate this class and listen on various topics to forward packets to this program. To get the state
of x device on the board, it shall recieve strings on the "aio_board" topic:
    format: 'CM'
Where C is a character refering to the device, M is the message attatched to the associated desired output.

(Rest of docstring is a copy/paste from Arduino end, copied 6/15/22, 08:23)
Firmware for All Input Output (AIO) PCB - Rev A

  This board interfaces the following sensors for the 2022 robosub vehicle:
            Sensor           |        Packet - Message
    -------------------------|----------------------------
    Autonomous Mode Button   |          0x1M - 0 Disable, 1 Enable, F Get
    Battery Monitor          |          0x2M - 0 Both Battery Sable,
                             |                 1 Battery #1 Voltage Unstable,
                             |                 2 Battery #2 Voltage Unstable,
                             |                 F Get
    Kill Mode Button         |          0x3M - 0 Disable, 1 Enable, F Get
    Leak Detection           |          0x4M - 0 No Leak, 1 Leak, F Get
    LED Strip                |           n/a
    Relay Mosfet Signal      |           n/a
    Torpedo Servo Motor      |          0x8M - 1 Torpedo #1 Empty,
                             |                 2 Torpedo #2 Empty,
                             |                 3 Both Empty,
                             |                 5 Torpedo #1 Fire,
                             |                 6 Torpedo #2 Fire,
                             |                 7 Both Fire,
                             |                 F Get
    Arm Gripper              |          0xAM - 0 Open, 1 Closed, F Get
Data stream behavior:
    Interrupt Packet - A packet sent by either device indicating new alert
    ----------------------------------------------------------------------
      format: 'i'0xNM'\n'
        'i' - ascii char i header byte representing interrupt packet
        Nibblets (4bits)
          0xN_ - Type of sensor making interrupt message
          0x_M - Message value attached to sensor
        '\n' - newline byte representing newline and end of packet

    Output Packet - A packet sent by either device as a response to the most
                    recent interrupt packet
    ----------------------------------------------------------------------
      format: 'o'0xNM\n
        'o' - ascii char o header byte representing response packet
        Nibblets (4bits)
          0xN_ - Type of sensor making response message
          0x_M - Message value attached to sensor
        '\n' - newline end byte representing newline and end of packet

ROS Behavior:
Published data to topic X contains a string in the following format:
iNM\0
Where:
i is the I/O state. 'i' means this is a new input to the computer. o means an acknowledged output.
N is the nmask character translated for ROS. See the ROS conversion tables in this file for more information.
M is the value associated with the nmask character. Ex. 2 on a battery monitor is Battery 2 voltage is unstable.
\0 is the null terminator used to end the string. (Not explicitly done in python strings)
"""
import sys
import time

import serial

import utils.scion_utils as scion_ut

# from ..utils import scion_utils as scion_ut

# Defining above spec in docstring
IN_HEADER_BYTE = ord('i')
OUT_HEADER_BYTE = ord('o')

# Lookup table for N byte conversion to array
nmask_dict_indicies = {
    10: 0,
    1: 1,
    2: 2,
    3: 3,
    4: 4,
    8: 5
}

# Lookup table for conversion from ROS
ros_conversion_table = {
    scion_ut.AIO_AUTO_NMASK_ROS: 1,  # autonomous button
    scion_ut.AIO_BAT_NMASK_ROS: 2,  # battery
    scion_ut.AIO_KILL_NMASK_ROS: 3,  # killswitch
    scion_ut.AIO_LEAK_NMASK_ROS: 4,  # leak
    scion_ut.AIO_TORPEDO_NMASK_ROS: 8,  # torpedo
    scion_ut.AIO_ARM_NMASK_ROS: 10,  # arm
}
# Lookup table for conversion to ROS
serial_converstion_table = {
    1: scion_ut.AIO_AUTO_NMASK_ROS,  # autonomous button
    2: scion_ut.AIO_BAT_NMASK_ROS,  # battery
    3: scion_ut.AIO_KILL_NMASK_ROS,  # killswitch
    4: scion_ut.AIO_LEAK_NMASK_ROS,  # leak
    8: scion_ut.AIO_TORPEDO_NMASK_ROS,  # torpedo
    10: scion_ut.AIO_ARM_NMASK_ROS,  # arm
}


class AIOWrapper:
    def __init__(self, device_name: str, timeout=0.0) -> None:
        self.dev_name = device_name
        self.timeout = timeout
        self.dev = serial.Serial(self.dev_name, baudrate=9600, timeout=timeout)
        self.last_line = ''

    def send_input_packet(self, nmask: str, value: int) -> None:
        """Sends input packet to device.
        """
        pack = self._gen_input_packet_int(nmask=nmask, val=value)
        self.dev.write(pack)

    def translate_recv_packet(self) -> str:
        """Translates read string into ROS format, return string
        """
        ret = ''
        ret = ret + (self.last_line[0])
        ret = ret + serial_converstion_table[(int(self.last_line[1:3], 16) >> 4)]
        ret = ret + hex(int(self.last_line[1:3], 16) & 15)[2:]  # Get N value, convert to hex, strip formatting
        return ret

    def read_device(self) -> any:
        out = self.dev.readline()
        if out == b'':
            return None
        print(f'[AIO UTILS] {out}')
        print(f'[AIO UTILS] {out[0:3]}')
        print(f'[AIO UTILS] {str(out[0:3])[2:-1]}')
        print(f'[AIO UTILS] {chr(int(str(out[0:3])[2:-1]))}')
        result = chr(int(f'{out[0:3]}'[2:-1]))
        print(f'[AIO UTILS] result | {result}')
        if out is not None or len(out) > 1:
            sep = ''
            res = ''
            i = 0
            while sep != ord('\n'):
                res = res + chr(out[i])
                i += 1
                try:
                    sep = out[i]
                except IndexError:
                    return None
            print(f'[AIO UTILS] res | {res}')
            result += res[:2] + hex(int(res))[2:]
            out = str(result)
            self.last_line = out
            print(f'[AIO UTILS] OUT | {out}')
            return out
        return None

    @staticmethod
    def _gen_input_packet_int(nmask: str, val: int) -> bytes:
        """Generates a 'get' packet using the nmask dictionary lookup.
        """
        ret = bytearray(3)
        nmask = nmask[:1]  # Strip other characters
        ret[0] = IN_HEADER_BYTE
        ret[1] = (ros_conversion_table[nmask[0]] << 4) + val
        ret[2] = ord('\n')
        return ret


def run_aio_test(dev_name: str):
    """Instantiates an AIO wrapper and validates various test protocols.
    """
    aiow = AIOWrapper(dev_name)
    # Test state of leak detection
    aiow.send_input_packet(nmask='l', value=15)
    time.sleep(0.1)
    print(aiow.read_device())
    print(aiow.translate_recv_packet())


if __name__ == '__main__':
    if len(sys.argv) > 1:  # Get USB mount location as an argument
        dev = sys.argv[1].replace(' ', '')
        run_aio_test(dev_name=dev)
    else:
        print('AIO Utils Error, sys.argv less than 2. (Did you add the device name?)')
        sys.exit(1)
