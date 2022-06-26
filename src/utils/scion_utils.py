import sys

import sensor.telemetry_linker as scion_tl

ROS_MODULE_NAME = 'rospy'

# AIO
AIO_AUTO_NMASK = 0x10
AIO_AUTO_OFF = 0x10
AIO_AUTO_ON = 0x11
AIO_AUTO_RX = 0x1E
AIO_AUTO_GET = 0x1F

AIO_BAT_NMASK = 0x20
AIO_BAT_STABLE = 0x20
AIO_BAT_WARN_1 = 0x21
AIO_BAT_WARN_2 = 0x22
AIO_BAT_WARN_BOTH = 0x23
AIO_BAT_RX = 0x2E
AIO_BAT_GET = 0x2F

AIO_KILL_NMASK = 0x30
AIO_KILL_OFF = 0x30
AIO_KILL_ON = 0x31
AIO_KILL_RX = 0x3E
AIO_KILL_GET = 0x3F

AIO_LEAK_NMASK = 0x40
AIO_LEAK_FALSE = 0x40
AIO_LEAK_TRUE = 0x41
AIO_LEAK_RX = 0x4E
AIO_LEAK_GET = 0x4F

AIO_TORPEDO_NMASK = 0x80
AIO_TORPEDO_1_EMPTY = 0x81
AIO_TORPEDO_2_EMPTY = 0x82
AIO_TORPEDO_BOTH_EMPTY = 0x83
AIO_TORPEDO_1_FIRE = 0x85
AIO_TORPEDO_2_FIRE = 0x86
AIO_TORPEDO_BOTH_FIRE = 0x87
AIO_TORPEDO_RX = 0x8E
AIO_TORPEDO_GET = 0x8F

AIO_ARM_NMASK = 0xA0
AIO_ARM_OPEN = 0xA0
AIO_ARM_CLOSE = 0xA1
AIO_ARM_RX = 0xAE
AIO_ARM_GET = 0xAF


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
                self.pitch = float(data[ind_1:ind_2])
                data = data[ind_2 + 2:]
                break
        # Roll
        ind_1 = 0
        for i in range(len(data)):
            if data[i] == 'Y':
                ind_2 = i
                self.roll = float(data[:ind_2])
                # Yaw
                self.yaw = float(data[ind_2 + 2:])


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
