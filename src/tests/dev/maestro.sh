#!/bin/bash
MAESTRO=$(ls /dev | grep ttyACM | wc -l)
if [[ $MAESTRO > 0 ]]; then
  echo "Maestro Device found at:"
  ls /dev | grep ttyACM | python3 start/maestro_dev_test.py
else
  echo "No maestro device detected"
fi