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
# Run test for USB devices in /dev
. start/enumerate.sh
# Test for maestro
MAESTRO=$(ls /dev | grep ttyACM | wc -l)
if [[ $MAESTRO > 0 ]]; then
  ls /dev | grep ttyACM | python3 start/maestro_dev_test.py
else
  echo "No maestro device detected"
fi
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
# Test for ROS running
ROS_RUNNING=$(ps -a | grep roscore | wc -l)
if [[ $ROS_RUNNING == 0 ]]; then
  roscore &
fi
# Test if masterprocess has been built
MP_BUILT=$(ls start/ | grep masterprocess.o | wc -l)
if [[ $MP_BUILT == 0 ]]; then
  cd start
  make
  cd ..
fi
# Start masterprocess
./start/masterprocess -c -d
