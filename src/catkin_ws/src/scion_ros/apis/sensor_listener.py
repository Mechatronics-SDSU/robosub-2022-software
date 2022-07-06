#!/usr/bin/env python3.8
"""Reads all sensor data on ROS using the telemetry linker and connects to a telemetry client.
"""

import rospy
from std_msgs.msg import String, Float64, Float32MultiArray
import socket

import utils.scion_utils as scion_ut
import sensor.telemetry_linker as scion_tl


def sensor_listener() -> None:
    """Listens to all ROS topics regarding sensors, updates telemetry linker class, forwards pickled data to clients.
    """
    '''Telemetry Linker. Not using shared memory because we listen to ROS directly. Since we only call pack_data, we
    never touch shared memory related functions and are allowed to pack pickles without causing errors. Maybe this
    should have been handled by a subclass that handles pickling, but that goes into OOP hell I don't care to pursue.
    '''
    linker = scion_tl.TelemetryLinker(use_shm=False)
    # ROS
    dw_ahrs = scion_ut.AHRSDataWrapper(debug=False)
    dw_depth = scion_ut.DepthDataWrapper(debug=False)
    dw_dvl = scion_ut.DVLDataWrapper(debug=False)
    dw_dvl_time = scion_ut.DVLTimeWrapper(debug=False)
    rospy.init_node('sensor_listener', anonymous=True)
    # Listen to all sensors
    rospy.Subscriber('ahrs_state', String, dw_ahrs.callback)
    rospy.Subscriber('depth_state', Float64, dw_depth.callback)
    rospy.Subscriber('dvl_data', Float32MultiArray, dw_dvl.callback)
    rospy.Subscriber('dvl_time', Float64, dw_dvl_time.callback)
    # Add more sensors here as they get added to ROS and update the telemetry linker
    # Frontend access
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', 50003))
        s.listen()
        print('Sensor API Listening...')
        conn, address = s.accept()
        # Main loop
        while True:
            result = conn.recvfrom(1024)[0]
            if result == b'1':
                # Get data from all data wrappers, load into linker
                linker.data[0] = dw_ahrs.pitch
                linker.data[1] = dw_ahrs.roll
                linker.data[2] = dw_ahrs.yaw
                linker.data[3] = float(dw_depth.depth)
                linker.data[4] = dw_dvl.dvl_x
                linker.data[5] = dw_dvl.dvl_y
                linker.data[6] = dw_dvl.dvl_z
                linker.data[7] = dw_dvl.dvl_mean
                linker.data[8] = dw_dvl_time.dvl_time
                # Pickle and send
                data = linker.pack_data()
                conn.sendall(data)


if __name__ == '__main__':
    sensor_listener()
