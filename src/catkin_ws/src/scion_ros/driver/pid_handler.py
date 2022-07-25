#!/usr/bin/env python3
"""ROS-connected pid driver
"""

from calendar import THURSDAY
import rospy
import sys
from std_msgs.msg import Float64, String, ByteMultiArray, Float32MultiArray
import control.scion_pid_controller as scion_pid
import utils.scion_utils as scion_ut
import json
import logging
import time
import numpy as np
from functools import partial

from utils.maestro_driver import MaestroDriver

PID_FETCH_HERTZ = 100

def shutdown_callback(thrusts: ByteMultiArray, pub: rospy.Publisher):
    thrusts.data = [0,0,0,0,0,0,0,0]
    pub.publish(thrusts)

def target_roll_callback(data, args) -> None:
    """Get targeted roll value. It should be already a float.
    """
    args = data

def target_pitch_callback(data, args) -> None:
    """Get targeted pitch value. It should be already a float.
    """
    args = data

def target_yaw_callback(data, args) -> None:
    """Get targeted yaw value. It should be already a float.
    """
    args = data

def target_depth_callback(data, args: float) -> None:
    """Get targeted depth value. It should be already a float.
    """
    args = float(data)

def target_vel_x_callback(data, args) -> None:
    """Get targeted velocity x value. It should be already a float.
    """
    args = data

def _angle_wrapped_error(angle_1, angle_2):

    error = angle_1 - angle_2

    if(error > np.pi):
        error = error - 2*np.pi
    elif(error < -1 * np.pi):
        error = error + 2*np.pi

    return(error)

def pid_driver(pid_name="str") -> None:
    #maestro = MaestroDriver(com_port=pid_name)

    # ROS
    dw_ahrs = scion_ut.AHRSDataWrapper(debug=False)
    dw_depth = scion_ut.DepthDataWrapper(debug=False)
    dw_dvl = scion_ut.DVLDataWrapper(debug=False)

    pid_pub = rospy.Publisher('pid_thrusts', ByteMultiArray, queue_size=8)
    rospy.init_node('pid_driver', anonymous=True)

    desired_depth = 0.0 #m

    desired_roll = 0.0 #rad
    desired_pitch = 0.0 #rad
    desired_yaw = 0*np.pi #rad

    desired_vel_x = 0.0 #m/s
    desired_vel_y = 0.0 #m/s
    desired_vel_z = 0.0 #m/s

    # Listen to all sensors
    rospy.Subscriber('ahrs_state', String, dw_ahrs.callback)
    rospy.Subscriber('depth_state', Float64, dw_depth.callback)
    rospy.Subscriber('dvl_data', Float32MultiArray, dw_dvl.callback)

    rospy.Subscriber('target_depth', Float64, partial(target_depth_callback, desired_depth))
    rospy.Subscriber('target_vel_x', Float64, target_vel_x_callback, desired_vel_x)
    rospy.Subscriber('target_roll', Float64, target_yaw_callback, desired_yaw)
    rospy.Subscriber('target_pitch', Float64, target_yaw_callback, desired_yaw)
    rospy.Subscriber('target_yaw', Float64, target_yaw_callback, desired_yaw)

    thrusts = ByteMultiArray()

    rospy.on_shutdown(partial(shutdown_callback, thrusts, pid_pub))

    #desired state for the control system to reach
    desired_pos_state = np.zeros(12)
    desired_pos_state[0] = desired_roll
    desired_pos_state[1] = desired_pitch
    desired_pos_state[2] = desired_yaw
    desired_pos_state[5] = desired_depth

    desired_vel_state = np.zeros(12)
    desired_vel_state[3] = desired_vel_x
    desired_vel_state[4] = desired_vel_y
    desired_vel_state[5] = desired_vel_z

    thrusts.data = [0, 0, 0, 0, 0, 0, 0, 0]

    pos_thrusts = np.zeros(8)
    vel_thrusts = np.zeros(8)

    #update rate of control system
    f = 100.0 #Hz
    dt = 1/f

    max_run_time = 3 #Time to run control system before exiting

    #load the pid_controller  parameters from a formatted json file
    with open(r'pid_params_1.json') as pid_param_file:
        pid_params = json.load(pid_param_file)

    # Init the controller
    pos_controller = scion_pid.Scion_Position_PID_Controller(pid_params)
    vel_controller = scion_pid.Scion_Velocity_PID_Controller(pid_params)

    curr_pos_state = np.zeros(12)
    prev_pos_state = np.zeros(12)

    curr_vel_state = np.zeros(12)
    prev_vel_state = np.zeros(12)

    rate = rospy.Rate(PID_FETCH_HERTZ)

    while not rospy.is_shutdown():

        curr_time = 0.0
        start_time = time.time()
        while(curr_time < max_run_time):

            #Get the state of the vehicle
            curr_pos_state[0] = dw_ahrs.roll
            curr_pos_state[1] = dw_ahrs.pitch
            curr_pos_state[2] = (dw_ahrs.yaw-180.0)*(np.pi/180.0)
            curr_pos_state[3] = 0.0
            curr_pos_state[4] = 0.0
            curr_pos_state[5] = float(dw_depth.depth)

            curr_pos_state[6] = _angle_wrapped_error(curr_pos_state[0], prev_pos_state[0]) / dt
            curr_pos_state[7] = _angle_wrapped_error(curr_pos_state[1], prev_pos_state[1]) / dt
            curr_pos_state[8] = _angle_wrapped_error(curr_pos_state[2], prev_pos_state[2]) / dt
            curr_pos_state[9] = (curr_pos_state[3] - prev_pos_state[3]) / dt
            curr_pos_state[10] = (curr_pos_state[4] - prev_pos_state[4]) / dt
            curr_pos_state[11] = (curr_pos_state[5] - prev_pos_state[5]) / dt

            prev_pos_state = np.copy(curr_pos_state)

            curr_vel_state[0] = 0.0
            curr_vel_state[1] = 0.0
            curr_vel_state[2] = 0.0
            curr_vel_state[3] = dw_dvl.dvl_x
            curr_vel_state[4] = 0.0 #dw_dvl.dvl_y
            curr_vel_state[5] = 0.0 #dw_dvl.dvl_z

            curr_vel_state[6] = _angle_wrapped_error(curr_vel_state[0], prev_vel_state[0]) / dt
            curr_vel_state[7] = _angle_wrapped_error(curr_vel_state[1], prev_vel_state[1]) / dt
            curr_vel_state[8] = _angle_wrapped_error(curr_vel_state[2], prev_vel_state[2]) / dt
            curr_vel_state[9] = (curr_vel_state[3] - prev_vel_state[3]) / dt
            curr_vel_state[10] = (curr_vel_state[4] - prev_vel_state[4]) / dt
            curr_vel_state[11] = (curr_vel_state[5] - prev_vel_state[5]) / dt

            prev_vel_state = np.copy(curr_vel_state)

            pos_thrusts, pos_errors, p_vel_errors = pos_controller.update(desired_pos_state, curr_pos_state, curr_vel_state, dt)
            vel_thrusts, vel_errors = vel_controller.update(desired_vel_state, curr_vel_state, dt)

            pos_thrusts = [int(i*1.0) for i in pos_thrusts]
            vel_thrusts = [int(j*1.0) for j in vel_thrusts]

            thrusts.data = np.add(pos_thrusts, vel_thrusts)

            print(thrusts.data)

            pid_pub.publish(thrusts)
            #maestro.set_thrusts(thrusts.data)

            time.sleep(dt)
            curr_time = (time.time() - start_time)
            rate.sleep()

        #maestro.set_thrusts([0,0,0,0,0,0,0,0])

if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        # Init pid controller
        pid_driver(dev)

    else:
        print('Error, argc not > 1. (Did you add the AHRS name when running this program?)')
        sys.exit(1)
