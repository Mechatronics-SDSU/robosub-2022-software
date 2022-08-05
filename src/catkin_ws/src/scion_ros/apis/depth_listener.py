#!/usr/bin/env python3
"""ROS-connected Depth Listener API
Currently hard coded for port 50006. May change in the future.
"""
import rospy
import socket
from std_msgs.msg import String

import utils.scion_utils as scion_ut


def depth_listener() -> None:
    # ROS
    dw = scion_ut.DataWrapper(debug=False)
    rospy.init_node('depth_listener', anonymous=True)
    rospy.Subscriber('depth_state', String, dw.callback)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', 50006))
        s.listen()
        print('Depth API Listening...')
        conn, address = s.accept()
        # Main loop
        while True:
            result = conn.recvfrom(1024)[0]
            if result == b'1':
                # Get ROS data
                data = bytes(dw.data)
                conn.sendall(data)


if __name__ == '__main__':
    depth_listener()
