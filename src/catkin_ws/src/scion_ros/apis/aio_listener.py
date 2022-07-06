#!/usr/bin/python3.8
"""API for GUI access to the AIO.
"""

import rospy
from std_msgs.msg import String
import socket

import utils.scion_utils as scion_ut
import aio.aio_linker as scion_aiol


def aio_listener() -> None:
    """Listens to ROS topics regarding the AIO, saves state in program memory.
    """
    linker = scion_aiol.AIOLinker(use_shm=False)


if __name__ == '__main__':
    aio_listener()
