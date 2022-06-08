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
# Change mod on devices
python3 start/dev_change.py
# ROS
roscore &
cd catkin_ws
catkin_make
source devel/setup.bash
cd ..
# Start masterprocess
./start/masterprocess -a
