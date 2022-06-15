import sys

import sensor.telemetry_linker as scion_tl

ROS_MODULE_NAME = 'rospy'
RIGHT_WIN_CONF = [25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 325, 350, 375, 400]


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
