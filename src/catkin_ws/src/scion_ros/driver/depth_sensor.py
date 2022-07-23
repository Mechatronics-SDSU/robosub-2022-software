#!/usr/bin/env python3
"""ROS-connected Depth sensor driver
"""
import rospy
import sys
from std_msgs.msg import Float64

import sensor.depth_sensor as scion_ds

DEPTH_FETCH_HERTZ = 10

def depth_driver(dev) -> None:
    print(f'DEPTH ON {dev}')
    depth = scion_ds.Depth(dev_name=dev, cal_offset=0.07)
    pub = rospy.Publisher('depth_state', Float64, queue_size=10)
    rospy.init_node('depth_driver', anonymous=True)
    rate = rospy.Rate(DEPTH_FETCH_HERTZ)
    while True:
        ds = depth.get_state()
        #print(f'SENSOR DRIVER SEES: {ds}')
        pub.publish(ds)
        rate.sleep()


if __name__ == '__main__':
    depth_driver(sys.argv[1])
