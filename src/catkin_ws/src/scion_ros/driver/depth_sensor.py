#!/usr/bin/env python3
"""ROS-connected Depth sensor driver
"""
import rospy
import sys
from std_msgs.msg import String

import sensor.depth_sensor as scion_ds

DEPTH_FETCH_HERTZ = 10


def depth_driver() -> None:
    depth = scion_ds.Depth()
    pub = rospy.Publisher('depth_state', String, queue_size=10)
    rospy.init_node('pub', anonymous=True)
    rate = rospy.Rate(DEPTH_FETCH_HERTZ)
    while True:
        ds = depth.get_state()
        print(f'SENSOR DRIVER SEES: {ds}')
        pub.publish(ds)
        rate.sleep()


if __name__ == '__main__':
    depth_driver()