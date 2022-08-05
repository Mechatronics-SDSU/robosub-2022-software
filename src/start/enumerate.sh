#!/bin/bash
# This script is awful and the worst code I've ever written. Do not repeat.
dmesg | grep usb | grep "SerialNumber:" | python3 start/match_dev.py
dmesg | grep usb | python3 start/devs_from_match.py
