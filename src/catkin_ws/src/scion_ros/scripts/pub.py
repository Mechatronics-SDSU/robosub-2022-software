#!/usr/bin/env python3
"""
Hello World publisher code
"""
import rospy
import sys
from std_msgs.msg import String


def main() -> None:
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('pub', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        message = f'Print test {rospy.get_time}'
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ERROR ROSpy Interrupt Exception")
        sys.exit(1)
