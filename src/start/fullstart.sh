#!/bin/bash
# PYTHONPATH check
echo "Starting script."
PATH_SET=$(export | grep PYTHONPATH | wc -l)
if [[ $PATH_SET == 0 ]]; then
  # PYTHONPATH is not set, add the directory to it
  source ./start/set_path.sh
else
  # PYTHONPATH is set, append the directory and keep whatever is in it
  source ./start/append_path.sh
fi
echo "Running as mechatronics." >> out.test
USB=0
while [ $USB -le 0 ]
do
  USB=$(ls /dev | grep ttyUSB | wc -l)
  echo "Test USB in iteration" >> out.test
  sleep 1
done
# Run test for USB devices in /dev
sudo -u mechatronics /home/mechatronics/sd/robosub-2022-software/src/start/enumerate.sh
# Test for maestro
# sleep 60
echo "Testing maestro." >> out.test
MAESTRO=0
while [ $MAESTRO -le 0 ]
do
  MAESTRO=$(ls /dev | grep ttyACM | wc -l)
  echo "Test Maestro in iteration" >> out.test
  sleep 1
done
# MAESTRO=$(ls /dev | grep ttyACM | wc -l)
#if [[ $MAESTRO > 0 ]]; then
ls /dev | grep ttyACM | sudo -u mechatronics python3 start/maestro_dev_test.py
#else
  #echo "No maestro device detected" >> out.test
#fi
# Change mod on devices so we can write to it
echo "Running dev change." >> out.test
python3 start/dev_change.py
echo "Changing account to Mechatronics..." >> out.test
sudo su mechatronics start/partial_start.sh
