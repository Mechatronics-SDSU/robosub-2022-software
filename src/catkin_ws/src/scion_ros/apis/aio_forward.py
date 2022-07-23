#!/usr/bin/env python3.8
"""AIO Forward client and server.
Server is on Scion and sends input to GUI.
Client is on GUI host computer and receives state from ROS.
"""

import rospy
from std_msgs.msg import String

import socket
import pickle
from multiprocessing import shared_memory as shm

import aio.aio_linker as scion_aiol
import utils.scion_utils as scion_ut


AIO_FORWARD_PORT = 50007
AIO_UPDATE_HERTZ = 50


def aio_forward_server() -> None:
    """Driver code for the aio forward server.
    """
    aiol = scion_aiol.AIOLinker()
    data = None
    try:
        aio_state_shm = shm.SharedMemory(create=True, size=6, name='aio_state_shm')
    except FileExistsError:
        aio_state_shm = shm.SharedMemory(name='aio_state_shm')
        aio_state_shm.unlink()
        aio_state_shm = shm.SharedMemory(create=True, size=6, name='aio_state_shm')
    try:
        state_send_shm = shm.SharedMemory(create=True, size=1, name='state_send_shm')
    except FileExistsError:
        state_send_shm = shm.SharedMemory(name='state_send_shm')
        state_send_shm.unlink()
        state_send_shm = shm.SharedMemory(create=True, size=1, name='state_send_shm')
    # ROS
    dw_kill = scion_ut.AIODataWrapperListener(debug=False, shm_index=0)
    dw_auto = scion_ut.AIODataWrapperListener(debug=False, shm_index=1)
    dw_bat = scion_ut.AIODataWrapperListener(debug=False, shm_index=2)
    dw_leak = scion_ut.AIODataWrapperListener(debug=False, shm_index=3)
    dw_weapons = scion_ut.AIODataWrapperListener(debug=False, shm_index=4)
    dw_arm = scion_ut.AIODataWrapperListener(debug=False, shm_index=5)
    rospy.Subscriber('aio_kill_state', String, dw_kill.callback)
    rospy.Subscriber('aio_auto_state', String, dw_auto.callback)
    rospy.Subscriber('aio_bat_state', String, dw_bat.callback)
    rospy.Subscriber('aio_leak_state', String, dw_leak.callback)
    rospy.Subscriber('aio_torp_state', String, dw_weapons.callback)
    rospy.Subscriber('aio_arm_state', String, dw_arm.callback)
    rospy.init_node('aio_forward')
    rate = rospy.Rate(AIO_UPDATE_HERTZ)
    # Socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', AIO_FORWARD_PORT))
        s.listen()
        s.settimeout(30)
        conn, addr = s.accept()
        s.settimeout(None)
        while True:
            if state_send_shm.buf[0] == 1:  # Something posted change, load into linker and send
                for i in range(len(aiol.data)):
                    aiol.data[i] = aio_state_shm.buf[i]
                conn.sendall(aiol.serialize())
                state_send_shm.buf[0] = 0
            rate.sleep()
        rate.sleep()


if __name__ == '__main__':
    print('Running AIO Forward server.')
    aio_forward_server()
