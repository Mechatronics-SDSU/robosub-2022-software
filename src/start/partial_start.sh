cd /home/mechatronics/sd/robosub-2022-software/src
echo "Changed to mechatronics." >> out2.test
# ROS
export PYTHONPATH=$PYTHONPATH:$(pwd)
cd catkin_ws
echo "changed to catkin" >> out2.test
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
echo "Started ROS if not running." >> out2.test
# Test if masterprocess has been built
MP_BUILT=$(ls start/ | grep masterprocess.o | wc -l)
if [[ $MP_BUILT == 0 ]]; then
  cd start
  make
  cd ..
fi
echo "Built masterprocess if not built." >> out2.test
# Start masterprocess
./start/masterprocess -i -d >> out2.test
