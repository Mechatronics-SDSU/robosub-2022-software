"""Scion's decision making process code.
"""
from turtle import forward
from xmlrpc.client import Boolean
import ai.mission_planner as scion_mis
import rospy
from std.msgs.msg import String, Float64, ByteMultiArray
import time
import sys
from functools import partial

import utils.scion_utils as scion_ut

HEURISTICS_UPDATE_HERTZ = 50
DEPTH_TOLERANCE = 0.1  # meters
PITCH_TOLERANCE = 2.0  # Degrees
ROLL_TOLERANCE = 1.0  # Degrees
YAW_TOLERANCE = 4.0  # Degrees

# Parser instructions for the mission planner.
COMMENT = '!'
COMPARE = '#'
BREAK = '*'
ITERABLE = '@'
BEGIN_REGISTER = '{'
END_REGISTER = '}'
BEGIN_ROS_PARAM = '('
END_ROS_PARAM = ')'
BEGIN_VAR = "\""
END_VAR = "\""
PYTHON_FLOAT = 'f64'
PYTHON_INT = 'i64'
PYTHON_LIST = 'list'
PYTHON_LIST_SEP = ','
BEGIN_PYTHON_LIST = '['
END_PYTHON_LIST = ']'
COMP_EQ = '=='
COMP_NE = '!='
COMP_LE = '<='
COMP_GE = '>='
COMP_LT = '<<'
COMP_GT = '>>'


class Heuristics:
    def __init__(self, missions: str):
        self.missions = scion_mis.MissionSystem(load_from_file=missions)
        self.instructions = self.parse_instructions()

    def parse_instructions(self) -> list:
        pass

    @staticmethod
    def run_instruction(self, instruction: str) -> bool:
        return True


class HeuristicsState:
    def __init__(self):
        self.last_roll_state = 0.0
        self.last_pitch_state = 0.0
        self.last_yaw_state = 0.0

def pid_callback(data, pub: rospy.Publisher, run: Boolean):
    """Callback of thruster outputs from PID system, contains a
    boolean to determine to forward to the pid thruster values to 
    the maestro subscriber
    """
    if run:
        pub.publish(data)
    else:
        pub.publish([0,0,0,0,0,0,0,0])

def shutdown_callback():
    """Prints heuristics is shutting down to stdout.
    """
    print('[AI] Shutting Down.')


def heuristics_test():
    """Build up the mission planner, loop over mission planner taking parameters.
    """
    h = Heuristics('ai/heuristics.cfg')
    print(h.missions.tree)


def heuristics_test_hard_coded():
    """Hard coded test to drive for 3 seconds while balanced on PID.
    """
    # Publisher topics
    target_depth_pub = rospy.Publisher('target_depth', Float64, queue_size=1)
    target_pitch_pub = rospy.Publisher('target_pitch', Float64, queue_size=1)
    target_roll_pub = rospy.Publisher('target_roll', Float64, queue_size=1)
    target_yaw_pub = rospy.Publisher('target_yaw', Float64, queue_size=1)
    target_vel_x_pub = rospy.Publisher('target_vel_x', Float64, queue_size=1)
    thruster_output = rospy.Publisher('thruster_output', ByteMultiArray, queue_size=1)

    accept_pid = True

    # Subscriber topics
    rospy.Subscriber('pid_thrusts', ByteMultiArray, partial(pid_callback, thruster_output, accept_pid))

    rospy.init_node('heuristics', anonymous=True)
    rospy.on_shutdown(shutdown_callback)
    rate = rospy.Rate(HEURISTICS_UPDATE_HERTZ)

    # Initialize state variable to describe mission planner
    state = 0
    
    curr_time = 0.0
    start_time = time.time()

    while not rospy.is_shutdown():
        if state == 0 & curr_time < 3:
            # Desired results for initial balance
            print("Initial state")
            target_depth_pub.publish(0.75)
            target_roll_pub.publish(0.0)
            target_pitch_pub.publish(0.0)
            target_yaw_pub.publish(0.0)  # CHANGE THIS DEPENDING ON SETUP
        
        elif state == 0:
            state = 1
            target_vel_x_pub.publish(0.5)
        
        elif state == 1 & curr_time >= 3 & curr_time < 6:
            print("Moving forward")
        
        elif state == 1:
            state = 2
            target_vel_x_pub.publish(0.0)
            target_depth_pub.publish(0.0)

        elif state == 2 & curr_time >= 6 & curr_time < 9:
            print("Rising to surface")

        elif state == 2:
            state = 3
            accept_pid = False

        elif state == 3:
            print("Task complete")
            sys.exit(1)

        curr_time = (time.time() - start_time)
        rate.sleep()


if __name__ == '__main__':
    #heuristics_test()
    heuristics_test_hard_coded()
