#!/usr/bin/env python3
"""ROS-connected AHRS API
Currently hard coded for port 50003. May change in the future.
"""
import rospy
import socket
from std_msgs.msg import String


class DataWrapper:
    def __init__(self, debug: bool) -> None:
        self.data = None
        self.debug = debug

    def callback(self, data) -> None:
        self.data = data.data
        if self.debug:
            print(f'API SEES: {data.data}')


def ahrs_listener() -> None:
    # ROS
    dw = DataWrapper(debug=False)
    rospy.init_node('ahrs_listener', anonymous=True)
    rospy.Subscriber('ahrs_state', String, dw.callback)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', 50003))
        s.listen()
        print('AHRS API Listening...')
        conn, address = s.accept()
        # Main loop
        while True:
            result = conn.recvfrom(1024)[0]
            if result == b'1':
                # Get ROS data
                data = bytes(dw.data)
                conn.sendall(data)


if __name__ == '__main__':
    ahrs_listener()
