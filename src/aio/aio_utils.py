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
    Arm Gripper              |          0x0M - 0 Open, 1 Closed, F Get
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
"""
import sys
import serial

import utils.scion_utils as scion_ut

# Defining above spec in docstring
IN_HEADER_BYTE = bytes(ord('i'))
OUT_HEADER_BYTE = bytes(ord('o'))

# Lookup table for N byte conversion to array
nmask_dict_indicies = {
    0: 0,
    1: 1,
    2: 2,
    3: 3,
    4: 4,
    8: 5
}
# Lookup table for ROS conversion
ros_conversion_table = {
    'r': 0,  # arm
    'a': 1,  # autonomous button
    'b': 2,  # battery
    'k': 3,  # killswitch
    'l': 4,  # leak
    't': 8,  # torpedo
}

NMASKS = [scion_ut.AIO_ARM_NMASK, scion_ut.AIO_AUTO_NMASK, scion_ut.AIO_BAT_NMASK, scion_ut.AIO_KILL_NMASK,
          scion_ut.AIO_LEAK_NMASK, scion_ut.AIO_TORPEDO_NMASK]
GET_REQ = [scion_ut.AIO_ARM_GET, scion_ut.AIO_AUTO_GET, scion_ut.AIO_BAT_GET, scion_ut.AIO_KILL_GET,
           scion_ut.AIO_LEAK_GET, scion_ut.AIO_TORPEDO_GET]


class AIOWrapper:
    def __init__(self, device_name: str):
        self.dev_name = device_name
        self.dev = serial.Serial(self.dev_name, 115200)
        self.last_line = ''

    def send_input_packet(self, nmask: int, value: int) -> None:
        """Sends input packet b'i<nmask><value>'
        """
        pass

    def translate_recv_packet(self) -> list:
        """Translates read string into specific ROS topic(s)
        """
        ret = []
        return ret

    def read_device(self) -> str:
        out = self.dev.readline()
        if out is not None:
            return str(out)

    def _gen_input_packet_ros(self) -> bytes:
        """Generates a 'get' packet using the ROS conversion table.
        """
        pass

    def _gen_input_packet_int(self) -> bytes:
        """Generates a 'get' packet using the nmask dictionary lookup.
        """
        pass


def run_aio_test(dev_name: str):
    """Instantiates an AIO wrapper and validates various test protocols.
    """
    aiow = AIOWrapper(dev_name)


if __name__ == '__main__':
    if len(sys.argv) > 1:  # Get USB mount location as an argument
        dev = sys.argv[1].replace(' ', '')
        run_aio_test(dev_name=dev)
    else:
        print('AIO Utils Error, sys.argv less than 2. (Did you add the device name?)')
        sys.exit(1)
