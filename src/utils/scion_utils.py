import sys

ROS_MODULE_NAME = 'rospy'


def scion_print(print_input: str):
    """Prints to ROS if rospy is imported.
    """
    if ROS_MODULE_NAME not in sys.modules:
        print(f'{ROS_MODULE_NAME} Not imported.')
    else:
        pub = rospy.Publisher('log', String, queue_size=1)
        pub.publish(print_input)


if __name__ == '__main__':
    print('Do not ruin scion_utils as main!')
    sys.exit(1)
