#!/bin/bash
# PYTHONPATH check
PATH_SET=$(export | grep PYTHONPATH | wc -l)
if [[ $PATH_SET == 0 ]]; then
  # PYTHONPATH is not set, add the directory to it
  source ./start/set_path.sh
else
  # PYTHONPATH is set, append the directory and keep whatever is in it
  source ./start/append_path.sh
fi
# Run test for devices in /dev
. start/enumerate.sh
# Change mod on devices so we can write to it
python3 start/dev_change.py
# ROS
cd catkin_ws
# Test if catkin has built
CATKIN_BUILT=$(ls | grep devel | wc -l)
if [[ $CATKIN_BUILT == 0 ]]; then
  catkin_make
fi
source devel/setup.bash
cd ..
roscore &
# Start masterprocess
# ./start/masterprocess -a
