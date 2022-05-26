"""
Subscriber test code
"""
import rospy
from std_msgs.msg import String


def callback(data) -> None:
    rospy.loginfo(rospy.get_caller_id() + f'I heard {data.data}')


def main() -> None:
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

