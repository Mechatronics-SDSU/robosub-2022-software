#!/usr/bin/python3.8
"""API for GUI access to the AIO.
"""

import rospy
from std_msgs.msg import String

import socket

import utils.scion_utils as scion_ut
import aio.aio_linker as scion_aiol


AIO_COMMAND_PORT = 50006
AIO_UPDATE_HERTZ = 50


def aio_listener() -> None:
    """Listens to ROS topics regarding the AIO, saves state in program memory.
    """
    # Linker copy of state of AIO
    aiol = scion_aiol.AIOLinker()
    # ROS
    dw_aio_list = [
        rospy.Publisher('aio_kill_change', String, queue_size=1),
        rospy.Publisher('aio_auto_change', String, queue_size=1),
        rospy.Publisher('aio_bat_change', String, queue_size=1),
        rospy.Publisher('aio_leak_change', String, queue_size=1),
        rospy.Publisher('aio_torp_change', String, queue_size=1),
        rospy.Publisher('aio_arm_change', String, queue_size=1)
    ]

    rospy.init_node('aio_listener', anonymous=True)
    rate = rospy.Rate(AIO_UPDATE_HERTZ)
    # Socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', AIO_COMMAND_PORT))
        s.listen()
        conn, addr = s.accept()
        print('Started AIO command server')
        while True:
            try:
                data = conn.recvfrom(1024)[0]
            except ConnectionAbortedError:
                print('Host closed from connection abort in AIO Listener Server.')
            if data is not None:  # Load pickled input
                res = aiol.from_serial_with_diff(data)  # Check for difference from last config
                if len(res[0]) > 0:
                    # Diff is > 0, send new messages on relevant ROS topic
                    for i in range(len(res[0])):
                        dw_aio_list[scion_aiol.aio_vals_from_char[res[0][i]]].publish(f'i{res[0][i]}{res[1][i]}\0')
            data = None
            rate.sleep()


if __name__ == '__main__':
    aio_listener()
