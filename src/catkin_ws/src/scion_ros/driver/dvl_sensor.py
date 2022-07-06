#!/usr/bin/env python3
"""ROS-connected DVL sensor driver
"""
import rospy
import sys
from std_msgs.msg import Float64, Float32MultiArray
import logging
import time

import sensor.dvl.dvl as scion_dvl

# Can be faster if needed
DVL_FETCH_HERTZ = 100


def dvl_driver(dvl_name: str) -> None:
    dvl = scion_dvl.Dvl(com=dvl_name)
    dvl_sample = scion_dvl.Dvl_sample()

    dvl.enter_command_mode()

    # Insert code for dvl setup configuration

    # Register callback function
    dvl.register_ondata_callback(scion_dvl.dvl_data_callback, dvl_sample)

    dvl.exit_command_mode()

    # Update to correct topic message
    pub_time = rospy.Publisher('dvl_time', Float64, queue_size=10)
    pub_data = rospy.Publisher('dvl_data', Float32MultiArray, queue_size=10)
    rospy.init_node('dvl_driver', anonymous=True)
    rate = rospy.Rate(DVL_FETCH_HERTZ)

    while True:
        # Trigger DVL data capture
        time = dvl_sample.get_time()
        data = dvl_sample.get_data()

        dvl.send_software_trigger()
        pub_time.publish(time)
        pub_data.publish(data)
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        dvl_driver(dvl_name=dev)
    else:
        print('Error, argc not > 1. (Did you add the DVL name when running this program?)')
        sys.exit(1)
