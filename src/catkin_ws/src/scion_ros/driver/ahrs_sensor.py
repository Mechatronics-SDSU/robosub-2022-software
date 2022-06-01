#!/usr/bin/env python3
"""ROS-connected AHRS sensor driver
"""
import rospy
import sys
import socket
from std_msgs.msg import String

import sensor.ahrs as scion_ahrs

def ahrs_driver() -> None:
    pass

if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        ahrs_driver(dvl_name=dev)
    else:
        print('Error, argc not > 1. (Did you add the DVL name when running this program?)')
        sys.exit(1)
