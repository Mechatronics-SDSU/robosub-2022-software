#!/usr/bin/env python3
"""ROS-connected pid driver
"""
from calendar import THURSDAY
import rospy
import sys
from std_msgs.msg import Float64, String, Float32MultiArray, MultiArrayDimension
import control.scion_pid_controller as scion_pid
import utils.scion_utils as scion_ut
import json
import logging
import time
import numpy as np
from utils.maestro_driver import MaestroDriver

PID_FETCH_HERTZ = 100

def target_depth_callback(data, args) -> None:
    """Get targeted depth value. It should be already a float.
    """
    args = data

def _angle_wrapped_error(angle_1, angle_2):
    
    error = angle_1 - angle_2
    
    if(error > np.pi):
        error = error - 2*np.pi
    elif(error < -1 * np.pi):
        error = error + 2*np.pi
    
    return(error)

def pid_driver(pid_name: str) -> None:
    maestro = MaestroDriver(com_port=pid_name)

    # ROS
    dw_ahrs = scion_ut.AHRSDataWrapper(debug=False)
    dw_depth = scion_ut.DepthDataWrapper(debug=False)
    dw_dvl = scion_ut.DVLDataWrapper(debug=False)

    pid_pub = rospy.Publisher('pid_thrusts', Float32MultiArray, queue_size=8)
    rospy.init_node('pid_driver', anonymous=True)

    # Listen to all sensors
    rospy.Subscriber('ahrs_state', String, dw_ahrs.callback)
    rospy.Subscriber('depth_state', Float64, dw_depth.callback)
    #rospy.Subscriber('dvl_data', Float32MultiArray, dw_dvl.callback)
    #rospy.Subscriber('target_depth', Float64, target_depth_callback, desired_depth)
    
    desired_depth = 0.3 #1.0m depth
    desired_roll = 0.0 #rad
    desired_pitch = 0.0 #rad
    desired_yaw = 0.0 #rad

    #desired state for the control system to reach
    desired_state = np.zeros(12)
    desired_state[0] = desired_roll
    desired_state[1] = desired_pitch
    desired_state[2] = desired_yaw
    desired_state[5] = desired_depth


    thrusts = Float32MultiArray()
    thrusts.data = []

    #update rate of control system
    f = 100.0 #Hz
    dt = 1/f

    max_run_time = 20 #Time to run control system before exiting

    #load the pid_controller  parameters from a formatted json file
    with open(r'pid_params_1.json') as pid_param_file:
        pid_params = json.load(pid_param_file)

    # Init the controller 
    controller = scion_pid.Scion_PID_Controller(pid_params)

    curr_state = np.zeros(12)  
    prev_state = np.zeros(12)
    
    curr_time = 0.0
    start_time = time.time()

    rate = rospy.Rate(PID_FETCH_HERTZ)
    while(curr_time < max_run_time):
            
        #Get the state of the vehicle
        curr_state[0] = dw_ahrs.roll
        curr_state[1] = dw_ahrs.pitch
        curr_state[2] = dw_ahrs.yaw
        curr_state[3] = 0.0
        curr_state[4] = 0.0
        curr_state[5] = float(dw_depth.depth)

        curr_state[6] = _angle_wrapped_error(curr_state[0], prev_state[0]) / dt
        curr_state[7] = _angle_wrapped_error(curr_state[1], prev_state[1]) / dt
        curr_state[8] = _angle_wrapped_error(curr_state[2], prev_state[2]) / dt
        curr_state[9] = (curr_state[3] - prev_state[3]) / dt
        curr_state[10] = (curr_state[4] - prev_state[4]) / dt
        curr_state[11] = (curr_state[5] - prev_state[5]) / dt

        prev_state = np.copy(curr_state)

        thrusts.data, errors = controller.update(desired_state, curr_state, dt)

        print(thrusts.data)

        pid_pub.publish(thrusts)
#        maestro.set_thrusts(thrusts)

        time.sleep(dt)
        curr_time = (time.time() - start_time)
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        dev = sys.argv[1].replace(' ', '')
        # Init pid controller 
        pid_driver(dev)

    else:
        print('Error, argc not > 1. (Did you add the AHRS name when running this program?)')
        sys.exit(1)
