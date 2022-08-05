#!/usr/bin/env python3
"""Quick and dirty logger for sensor data.
"""

import rospy
import time
from std_msgs.msg import String


class DataWrapper:
    def __init__(self, debug: bool) -> None:
        self.data = None
        self.debug = debug
        self.fd = open('depth.txt', 'a')

    def callback(self, data) -> None:
        self.data = data.data
        if self.debug:
            print(f'{type(data)}: {data}')
            self.fd = open('depth.txt', 'a')
            self.fd.write(f'{str(data)} T:{time.time()}\n')
            self.fd.close()


def depth_logger() -> None:
    dw = DataWrapper(debug=True)
    rospy.init_node('depth_logger', anonymous=True)
    rospy.Subscriber('depth_state', String, dw.callback)
    rate = rospy.Rate(100)
    while True:
        rate.sleep()


if __name__ == '__main__':
    depth_logger()
