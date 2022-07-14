#!/usr/bin/env python3.8
import sys
import struct
from multiprocessing import shared_memory as shm

import sensor.telemetry_linker as scion_tl

ROS_MODULE_NAME = 'rospy'
RIGHT_WIN_CONF = [25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 325, 350, 375, 400]

# AIO
AIO_AUTO_NMASK_ROS = 'a'
AIO_AUTO_NMASK = 0x10
AIO_AUTO_OFF = 0x0
AIO_AUTO_ON = 0x1
AIO_AUTO_RX = 0xE
AIO_AUTO_GET = 0xF

AIO_BAT_NMASK_ROS = 'b'
AIO_BAT_NMASK = 0x20
AIO_BAT_STABLE = 0x0
AIO_BAT_WARN_1 = 0x1
AIO_BAT_WARN_2 = 0x2
AIO_BAT_WARN_BOTH = 0x3
AIO_BAT_RX = 0xE
AIO_BAT_GET = 0xF

AIO_KILL_NMASK_ROS = 'k'
AIO_KILL_NMASK = 0x30
AIO_KILL_OFF = 0x0
AIO_KILL_ON = 0x1
AIO_KILL_RX = 0xE
AIO_KILL_GET = 0xF

AIO_LEAK_NMASK_ROS = 'l'
AIO_LEAK_NMASK = 0x40
AIO_LEAK_FALSE = 0x0
AIO_LEAK_TRUE = 0x1
AIO_LEAK_RX = 0xE
AIO_LEAK_GET = 0xF

AIO_TORPEDO_NMASK_ROS = 'w'
AIO_TORPEDO_NMASK = 0x80
AIO_TORPEDO_1_EMPTY = 0x1
AIO_TORPEDO_2_EMPTY = 0x2
AIO_TORPEDO_BOTH_EMPTY = 0x3
AIO_TORPEDO_1_FIRE = 0x5
AIO_TORPEDO_2_FIRE = 0x6
AIO_TORPEDO_BOTH_FIRE = 0x7
AIO_TORPEDO_RX = 0xE
AIO_TORPEDO_GET = 0xF

AIO_ARM_NMASK_ROS = 'r'
AIO_ARM_NMASK = 0xA0
AIO_ARM_OPEN = 0x0
AIO_ARM_CLOSE = 0x1
AIO_ARM_RX = 0xE
AIO_ARM_GET = 0xF


class DataWrapper:
    """Handles data sent from ROS and all related parsing and type casting.
    """
    def __init__(self, debug: bool) -> None:
        self.data = None
        self.debug = debug

    def callback(self, data) -> None:
        self.data = data.data
        if self.debug:
            print(f'API SEES: {data.data}')


class AIODataWrapper(DataWrapper):
    def __init__(self, debug: bool) -> None:
        super().__init__(debug)
        self.data = None

    def callback(self, data) -> None:
        self.data = data.data
        if self.debug:
            print(f'AIODataWrapper sees: {data.data}')


class AIODataWrapperListener(DataWrapper):
    def __init__(self, debug: bool, shm_index: int) -> None:
        super().__init__(debug)
        self.data = None
        self.shm_index = shm_index
        self.aio_state_shm = shm.SharedMemory(name='aio_state_shm')
        self.state_send_shm = shm.SharedMemory(name='state_send_shm')

    def callback(self, data) -> None:
        self.data = data.data
        self.state_send_shm.buf[0] = 1
        self.aio_state_shm.buf[self.shm_index] = data.data
        if self.debug:
            print(f'AIODataWrapperListener sees: {data.data}')


class DVLDataWrapper(DataWrapper):
    """Specific wrapper for the DVL data.
    """
    def __init__(self, debug: bool):
        super().__init__(debug)
        self.dvl_x = scion_tl.TELEMETRY_DEFAULT_DATA[4]
        self.dvl_y = scion_tl.TELEMETRY_DEFAULT_DATA[5]
        self.dvl_z = scion_tl.TELEMETRY_DEFAULT_DATA[6]
        self.dvl_mean = scion_tl.TELEMETRY_DEFAULT_DATA[7]

    def callback(self, data) -> None:
        if self.debug:
            print(data)
        self.dvl_x = struct.pack('>1f', data[0])
        self.dvl_y = struct.pack('>1f', data[1])
        self.dvl_z = struct.pack('>1f', data[2])
        self.dvl_mean = struct.pack('>1f', data[3])


class DVLTimeWrapper(DataWrapper):
    """Specific wrapper for the DVL timestamps.
    """
    def __init__(self, debug: bool):
        super().__init__(debug)
        self.dvl_time = scion_tl.TELEMETRY_DEFAULT_DATA[8]

    def callback(self, data) -> None:
        if self.debug:
            print(data)
        self.dvl_time = float(data)


class DepthDataWrapper(DataWrapper):
    """Specific wrapper for the Depth data packets.
    """
    def __init__(self, debug: bool):
        super().__init__(debug)
        self.depth = scion_tl.TELEMETRY_DEFAULT_DATA[3]

    def callback(self, data) -> None:
        """Get Depth data. It should be already a float.
        """
        self.depth = data


class AHRSDataWrapper(DataWrapper):
    """Specific wrapper for the AHRS data packets. Perform translation into correct data type.
    """
    def __init__(self, debug: bool) -> None:
        super().__init__(debug)
        self.pitch = scion_tl.TELEMETRY_DEFAULT_DATA[0]
        self.roll = scion_tl.TELEMETRY_DEFAULT_DATA[1]
        self.yaw = scion_tl.TELEMETRY_DEFAULT_DATA[2]

    def callback(self, data) -> None:
        """Parse AHRS data into floats
        String looks like:
        "P|<float>R|<float>Y|<float>\n"
        """
        data = str(data)
        data = data.strip()  # Remove newline
        ind_1 = 0
        ind_2 = 0
        # Pitch
        for i in range(len(data)):
            if data[i] == 'P':
                ind_1 = i+2
            elif (data[i] == 'R') and (ind_1 != 0):
                ind_2 = i
                self.pitch = data[ind_1:ind_2]
                if self.pitch == 'None':
                    self.pitch = 0.0
                else:
                    self.pitch = float(self.pitch)
                data = data[ind_2 + 2:]
                break
        # Roll
        ind_1 = 0
        for i in range(len(data)):
            if data[i] == 'Y':
                ind_2 = i
                self.roll = data[:ind_2]
                if self.roll == 'None':
                    self.roll = 0.0
                else:
                    self.roll = float(self.roll)
                # Yaw
                self.yaw = data[ind_2 + 2:-1]
                if self.yaw == 'None':
                    self.yaw = 0.0
                else:
                    self.yaw = float(self.yaw)


def scion_print(print_input: str):
    """Prints to ROS if rospy is imported.
    """
    if ROS_MODULE_NAME not in sys.modules:
        print(f'{ROS_MODULE_NAME} Not imported.')
    else:
        pub = rospy.Publisher('log', String, queue_size=1)
        pub.publish(print_input)


if __name__ == '__main__':
    print('Do not ruin scion_utils as main!')
    sys.exit(1)
