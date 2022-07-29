"""Scion's decision making process code.
"""
import ai.mission_planner as scion_mis
import rospy
from std.msgs.msg import String, Float64, ByteMultiArray

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
    thruster_output = rospy.Publisher('thruster_output', ByteMultiArray, queue_size=1)
    # Subscriber topics

    rospy.init_node('heuristics', anonymous=True)
    rospy.on_shutdown(shutdown_callback)
    rate = rospy.Rate(HEURISTICS_UPDATE_HERTZ)
    # START AI
    # Desired results for initial balance
    target_depth_pub.publish(0.75)
    target_roll_pub.publish(0.0)
    target_pitch_pub.publish(0.0)
    target_yaw_pub.publish(0.0)  # CHANGE THIS DEPENDING ON SETUP
    # Wait until within tolerance

    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    heuristics_test()
