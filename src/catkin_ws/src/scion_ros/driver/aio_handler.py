#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import sys
import aio.aio_utils as scion_aio

AIO_FETCH_DELAY = 50


def aio_handler(aio_name: str) -> None:
    # Set up AIO
    aio = scion_aio.AIOWrapper(device_name=aio_name)
    rate = rospy.Rate()
    # Main loop
    while True:
        # Check for new messages

        rate.sleep(AIO_FETCH_DELAY)
        # Check ROS for latest messages

        rate.sleep(AIO_FETCH_DELAY)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        aio_handler(aio_name=dev)
    else:
        print('Error: argc not > 1. ')
