#!/usr/bin/env python3
"""Writes out maestro name from stdin.
"""
import fileinput


def write_maestro():
    outnum = []
    for line in fileinput.input():
        line = line.strip()  # Remove newline
        outnum.append(int(line[-1:]))  # Only care about last number
    outnum.sort()  # Sort to guarantee index 0 is lowest
    # Write location of maestro to file
    f = open('start/current_devices.cfg', 'a')
    f_1 = open('start/current_device_names.cfg', 'a')
    f.write(f'/dev/ttyACM{outnum[0]}\n')
    f_1.write('Maestro\n')
    f.close()
    f_1.close()


if __name__ == '__main__':
    write_maestro()
