"""
ROS message handling class for the CAN driver. Spec to be added.
"""

class CanRos():
    def __init__(self):
        self.ros_thread = None

        print(f'Started CanRos on ROS Thread {self.ros_thread}')

if __name__ == '__main__':
    CR = CanRos()
