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
        self.fd = open('ahrs.txt', 'a')

    def callback(self, data) -> None:
        self.data = data.data
        if self.debug:
            print(data)
            self.fd.write(f'{str(data)} T:{time.time()}\n')


def ahrs_logger() -> None:
    dw = DataWrapper(debug=True)
    rospy.init_node('ahrs_logger', anonymous=True)
    rospy.Subscriber('ahrs_state', String, dw.callback)
    rate = rospy.Rate(100)
    while True:
        rate.sleep()


if __name__ == '__main__':
    ahrs_logger()
