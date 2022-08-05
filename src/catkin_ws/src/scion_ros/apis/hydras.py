"""Hydrophone driver for ROS.
Connects to the Raspberry Pi hydrophone computer using a socket server and forwards packets to ROS.

"""
import socket
import rospy
from std_msgs.msg import String


hydra_port = 50010

def hydra_ros_driver(port: int)-> None:
    # ROS
    rate = rospy.Rate(50)
    # dw = DataWrapper(debug=False)
    pub = rospy.Publisher('hydras_state', String, queue_size=1)
    rospy.init_node('hydras', anonymous=True)
    # Server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', port))
        s.listen(5)
        conn, addr = s.accept()
        while True:
            # Get data from Pi
            data = s.recv(1024)
            # Publish to ROS
            pub.publish(str(data[2:-1])])
            data = None
            rate.sleep()


if __name__ == '__main__':
    hydra_ros_driver(port=hydra_port)
