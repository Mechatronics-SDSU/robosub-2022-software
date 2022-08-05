#!/usr/bin/env python3
"""ROS-connected pid driver
"""

from calendar import THURSDAY
from distutils.log import debug
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

PID_FETCH_HERTZ = 100

def shutdown_callback(thrusts: ByteMultiArray, pub: rospy.Publisher):
    thrusts.data = [0,0,0,0,0,0,0,0]
    pub.publish(thrusts)

def _angle_wrapped_error(angle_1, angle_2):

    error = angle_1 - angle_2

    if(error > np.pi):
        error = error - 2*np.pi
    elif(error < -1 * np.pi):
        error = error + 2*np.pi

    return(error)

def pid_driver() -> None:

    # ROS
    dw_ahrs = scion_ut.AHRSDataWrapper(debug=False)
    dw_depth = scion_ut.DepthDataWrapper(debug=False)
    dw_dvl = scion_ut.DVLDataWrapper(debug=False)

    pid_pub = rospy.Publisher('pid_thrusts', ByteMultiArray, queue_size=8)
    rospy.init_node('pid_driver', anonymous=True)

    dw_target_roll = scion_ut.DataWrapper(debug=False)
    dw_target_pitch = scion_ut.DataWrapper(debug=False)
    dw_target_yaw = scion_ut.DataWrapper(debug=False)
    dw_target_depth = scion_ut.DataWrapper(debug=False)
    dw_target_vel_x = scion_ut.DataWrapper(debug=False)
    dw_target_vel_y = scion_ut.DataWrapper(debug=False)
    dw_target_vel_z = scion_ut.DataWrapper(debug=False)
    
    dw_target_roll.data = 0.0 #rad
    dw_target_pitch.data = 0.0 #rad
    dw_target_yaw.data = 13.0 * (np.pi/180) #rad
    dw_target_depth.data = 0.0 #m

    dw_target_vel_x.data = 1.0 #m/s
    dw_target_vel_y.data = 0.0 #m/s
    dw_target_vel_z.data = 0.0 #m/s

    # Listen to all sensors
    rospy.Subscriber('ahrs_state', String, dw_ahrs.callback)
    rospy.Subscriber('depth_state', Float64, dw_depth.callback)
    rospy.Subscriber('dvl_data', Float32MultiArray, dw_dvl.callback)

    rospy.Subscriber('target_roll', Float64, dw_target_roll.callback)
    rospy.Subscriber('target_pitch', Float64, dw_target_pitch.callback)
    rospy.Subscriber('target_yaw', Float64, dw_target_yaw.callback)
    rospy.Subscriber('target_depth', Float64, dw_target_depth.callback)
    rospy.Subscriber('target_vel_x', Float64, dw_target_vel_x.callback)

    thrusts = ByteMultiArray()

    rospy.on_shutdown(partial(shutdown_callback, thrusts, pid_pub))
    print('[PID] Registered shutdown callback...')
    thrusts.data = [0, 0, 0, 0, 0, 0, 0, 0]

    pos_thrusts = np.zeros(8)
    vel_thrusts = np.zeros(8)

    #update rate of control system
    f = 100.0 #Hz
    dt = 1/f
    print('[PID] Declaring max runtime...')
    max_run_time = 3 #Time to run control system before exiting

    #load the pid_controller  parameters from a formatted json file
    print('[PID] Loading PID parameters...')
    with open('catkin_ws/src/scion_ros/driver/pid_params_1.json', 'r') as pid_param_file:
        pid_params = json.load(pid_param_file)
    print('[PID] Loaded PID parameters.')
    # Init the controller
    pos_controller = scion_pid.Scion_Position_PID_Controller(pid_params)
    vel_controller = scion_pid.Scion_Velocity_PID_Controller(pid_params)

    curr_pos_state = np.zeros(12)
    prev_pos_state = np.zeros(12)

    curr_vel_state = np.zeros(12)
    prev_vel_state = np.zeros(12)

    desired_pos_state = np.zeros(12)
    desired_vel_state = np.zeros(12)
    print('[PID] Establishing rate...')
    rate = rospy.Rate(PID_FETCH_HERTZ)
    print('[PID] Starting PID Handler')
    while not rospy.is_shutdown():
        print('[PID] Running PID...')
        #desired state for the control system to reach
    
        desired_pos_state[0] = dw_target_roll.data
        desired_pos_state[1] = dw_target_pitch.data
        desired_pos_state[2] = dw_target_yaw.data
        desired_pos_state[5] = dw_target_depth.data

        desired_vel_state[3] = dw_target_vel_x.data
        desired_vel_state[4] = dw_target_vel_y.data
        desired_vel_state[5] = dw_target_vel_z.data

        curr_time = 0.0
        start_time = time.time()
        while(curr_time < max_run_time):
            print('[PID] Iteration of PID in loop')
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

            pid_pub.publish(thrusts)

            time.sleep(dt)
            curr_time = (time.time() - start_time)
            rate.sleep()


if __name__ == '__main__':
    # Init pid controller
    print('[PID] Starting Program...')
    pid_driver()
