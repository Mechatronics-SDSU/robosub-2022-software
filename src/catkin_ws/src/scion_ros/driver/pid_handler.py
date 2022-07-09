#!/usr/bin/env python3
"""ROS-connected AHRS sensor driver
"""
import rospy
import sys
from std_msgs.msg import ByteMultiArray
import logging
import time

# import control.pid_controller as scion_pid


PID_FETCH_HERTZ = 100


def pid_driver(pid_name: str) -> None:
    # pid = scion_pid.SpartonAHRSDataPackets(_com_port=ahrs_name)
    pub = rospy.Publisher('pid_movement', ByteMultiArray, queue_size=10)
    rospy.init_node('pid_driver', anonymous=True)
    rate = rospy.Rate(PID_FETCH_HERTZ)
    while True:
        # yaw = ahrs.get_true_heading()
        # pitch, roll = ahrs.get_pitch_roll()
        # if yaw is not None:
        #     #  print(f'DRIVER SENDING: P|{pitch}R|{roll}Y|{yaw}')
        #     pub.publish(f'P|{pitch}R|{roll}Y|{yaw}')
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        # ahrs_driver(ahrs_name=dev)
        # Init pid controller
    else:
        print('Error, argc not > 1. (Did you add the AHRS name when running this program?)')
        sys.exit(1)