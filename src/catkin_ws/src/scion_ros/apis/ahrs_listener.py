#!/usr/bin/env python3
"""ROS-connected AHRS API
Currently hard coded for port 50003. May change in the future.
"""
import rospy
import sys
import socket
from std_msgs.msg import String


class DataWrapper:
    def __init__(self):
        self.data = None

    def callback(self, data) -> None:
        self.data = data.data


def ahrs_listener() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', 50003))
        s.listen()
        print('AHRS API Listening...')
        conn, address = s.accept()
        # ROS
        dw = DataWrapper()
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('ahrs_state', String, dw.callback)
        # Main loop
        while True:
            result = conn.recvfrom(1024)[0]
            if result == b'1':
                # Get ROS data
                data = bytes(dw.data)
                conn.sendall(data)


if __name__ == '__main__':
    ahrs_listener()
    """
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        ahrs_driver(dvl_name=dev)
    else:
        print('Error, argc not > 1. (Did you add the DVL name when running this program?)')
        sys.exit(1)
    """
