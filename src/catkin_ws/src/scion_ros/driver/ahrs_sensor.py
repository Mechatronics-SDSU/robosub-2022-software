#!/usr/bin/env python3
"""ROS-connected AHRS sensor driver
"""
import rospy
import sys
from std_msgs.msg import String
import logging
import time

import sensor.ahrs as scion_ahrs


AHRS_FETCH_HERTZ = 100


def ahrs_driver(ahrs_name: str) -> None:
    ahrs = scion_ahrs.SpartonAHRSDataPackets(_com_port=ahrs_name)
    pub = rospy.Publisher('ahrs_state', String, queue_size=10)
    rospy.init_node('pub', anonymous=True)
    rate = rospy.Rate(AHRS_FETCH_HERTZ)
    while True:
        yaw = ahrs.get_true_heading()
        pitch, roll = ahrs.get_pitch_roll()
        print(f'DRIVER SENDING: P|{pitch}R|{roll}Y|{yaw}')
        pub.publish(f'P|{pitch}R|{roll}Y|{yaw}')
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        ahrs_driver(ahrs_name=dev)
    else:
        print('Error, argc not > 1. (Did you add the AHRS name when running this program?)')
        sys.exit(1)
