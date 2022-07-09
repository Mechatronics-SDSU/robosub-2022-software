#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String

import sys
import utils.scion_utils as scion_ut
import aio.aio_utils as scion_aio

AIO_FETCH_DELAY = 50
AIO_TIMEOUT_MS = 0.005


def aio_handler(aio_name: str, debug=False) -> None:
    # Set up AIO
    aio = scion_aio.AIOWrapper(device_name=aio_name, timeout=AIO_TIMEOUT_MS)
    # ROS topics
    aio_auto_pub = rospy.Publisher('aio_auto_state', String, queue_size=1)
    aio_auto_dw = scion_ut.DataWrapper(debug=debug)
    rospy.Subscriber('aio_auto_change', String, aio_auto_dw.callback)
    aio_bat_pub = rospy.Publisher('aio_bat_state', String, queue_size=1)
    aio_bat_dw = scion_ut.DataWrapper(debug=debug)
    rospy.Subscriber('aio_bat_change', String, aio_bat_dw.callback)
    aio_kill_pub = rospy.Publisher('aio_kill_state', String, queue_size=1)
    aio_kill_dw = scion_ut.DataWrapper(debug=debug)
    rospy.Subscriber('aio_kill_change', String, aio_kill_dw.callback)
    aio_leak_pub = rospy.Publisher('aio_leak_state', String, queue_size=1)
    aio_leak_dw = scion_ut.DataWrapper(debug=debug)
    rospy.Subscriber('aio_leak_change', String, aio_leak_dw.callback)
    aio_torp_pub = rospy.Publisher('aio_torp_state', String, queue_size=1)
    aio_torp_dw = scion_ut.DataWrapper(debug=debug)
    rospy.Subscriber('aio_torp_change', String, aio_torp_dw.callback)
    aio_arm_pub = rospy.Publisher('aio_arm_state', String, queue_size=1)
    aio_arm_dw = scion_ut.DataWrapper(debug=debug)
    rospy.Subscriber('aio_arm_change', String, aio_arm_dw.callback)
    rospy.init_node('aio_handler', anonymous=True)
    rate = rospy.Rate(AIO_FETCH_DELAY)
    # Main loop
    while True:
        # Check for new messages
        ret = aio.read_device()
        if ret is not None:  # Recieved new packet
            ret = aio.translate_recv_packet()
            # Translate and send to relevant ROS topic
            if ret[1] == scion_ut.AIO_AUTO_NMASK_ROS:
                aio_auto_pub.publish(ret)
            elif ret[1] == scion_ut.AIO_BAT_NMASK_ROS:
                aio_bat_pub.publish(ret)
            elif ret[1] == scion_ut.AIO_KILL_NMASK_ROS:
                aio_kill_pub.publish(ret)
            elif ret[1] == scion_ut.AIO_LEAK_NMASK_ROS:
                aio_leak_pub.publish(ret)
            elif ret[1] == scion_ut.AIO_TORPEDO_NMASK_ROS:
                aio_torp_pub.publish(ret)
            elif ret[1] == scion_ut.AIO_ARM_NMASK_ROS:
                aio_arm_pub.publish(ret)
        rate.sleep()
        # Check for latest messages in callbacks
        '''
        Order of the following if/else block matters. Things ordered higher in the block
        will "take priority" and be done first.
        Order of operations:
        - Cast to python string (may have parsing issues)
        - Translate the input packet (get the nmask and value)
        - Send the input packet
        - Set data to None type
        '''
        if aio_kill_dw.data is not None:
            data = str(aio_kill_dw.data)
            nmask = data[1]
            val = int(data[2], 16)
            if debug:
                print(f'AIO kill sees: callback_data={data} | nmask={nmask} | val={val}')
            aio.send_input_packet(nmask=nmask, value=val)
            aio_kill_dw.data = None
        elif aio_leak_dw.data is not None:
            data = str(aio_leak_dw.data)
            nmask = data[1]
            val = int(data[2], 16)
            if debug:
                print(f'AIO leak sees: callback_data={data} | nmask={nmask} | val={val}')
            aio.send_input_packet(nmask=nmask, value=val)
            aio_leak_dw.data = None
        elif aio_bat_dw.data is not None:
            data = str(aio_bat_dw.data)
            nmask = data[1]
            val = int(data[2], 16)
            if debug:
                print(f'AIO bat sees: callback_data={data} | nmask={nmask} | val={val}')
            aio.send_input_packet(nmask=nmask, value=val)
            aio_bat_dw.data = None
        elif aio_auto_dw.data is not None:
            data = str(aio_auto_dw.data)
            nmask = data[1]
            val = int(data[2], 16)
            if debug:
                print(f'AIO auto sees: callback_data={data} | nmask={nmask} | val={val}')
            aio.send_input_packet(nmask=nmask, value=val)
            aio_auto_dw.data = None
        elif aio_torp_dw.data is not None:
            data = str(aio_torp_dw.data)
            nmask = data[1]
            val = int(data[2], 16)
            if debug:
                print(f'AIO torp sees: callback_data={data} | nmask={nmask} | val={val}')
            aio.send_input_packet(nmask=nmask, value=val)
            aio_torp_dw.data = None
        elif aio_arm_dw.data is not None:
            data = str(aio_arm_dw.data)
            nmask = data[1]
            val = int(data[2], 16)
            if debug:
                print(f'AIO arm sees: callback_data={data} | nmask={nmask} | val={val}')
            aio.send_input_packet(nmask=nmask, value=val)
            aio_arm_dw.data = None
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        aio_handler(aio_name=dev, debug=True)
    else:
        print('Error: argc not > 1. ')
