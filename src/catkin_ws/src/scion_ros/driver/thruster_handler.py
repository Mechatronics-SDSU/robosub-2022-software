#!/usr/bin/env python3
"""ROS-connected Maestro thruster driver
"""
import rospy
import sys
from std_msgs.msg import ByteMultiArray
import logging
import time
from functools import partial

import utils.maestro_driver as scion_thrusters

THRSUTER_FETCH_HERTZ = 10

def shutdown_callback(maestro: scion_thrusters.MaestroDriver):
    maestro.set_thrusts([0,0,0,0,0,0,0,0])

def thruster_callback(data, maestro: scion_thrusters.MaestroDriver):
    maestro.set_thrusts(data.data)

def thruster_driver(maestro_port: str) -> None:

    thrusters = scion_thrusters.MaestroDriver(com_port=maestro_port, baud_rate=9600)

    rospy.init_node('thuster_handler', anonymous=True)
    #rospy.Subscriber('thruster_output', ByteMultiArray, thruster_callback, thrusters)
    rospy.Subscriber('pid_thrusts', ByteMultiArray, thruster_callback, thrusters)

    rospy.on_shutdown(partial(shutdown_callback, thrusters))

    rate = rospy.Rate(THRSUTER_FETCH_HERTZ)

    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        thruster_driver(maestro_port=dev)
    else:
        print('Error, argc not > 1. (Did you add the maestro name when running this program?)')
        sys.exit(1)
