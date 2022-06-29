#!/usr/bin/env python3
"""ROS-connected DVL sensor driver
"""
import rospy
import sys
from std_msgs.msg import String
import logging
import time

import sensor.dvl.dvl as scion_dvl

# Can be faster if needed
DVL_FETCH_HERTZ = 100

def dvl_driver(dvl_name: str) -> None:
    dvl = scion_dvl.Dvl(com=dvl_name)

    dvl.enter_command_mode()

    # Insert code for dvl setup configuration

    # Register callback function
    dvl.register_ondata_callback(scion_dvl.dvl_data_callback, None)

    dvl.exit_command_mode()

    # Update to correct topic message
    # pub = rospy.Publisher('dvl_state', String, queue_size=10)
    # rospy.init_node('pub', anonymous=True)
    rate = rospy.Rate(DVL_FETCH_HERTZ)

    while True:
        # Trigger DVL data capture
        dvl.send_software_trigger()
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        dvl_driver(dvl_name=dev)
    else:
        print('Error, argc not > 1. (Did you add the DVL name when running this program?)')
        sys.exit(1)