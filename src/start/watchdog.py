#!/usr/bin/env python3.8
"""Monitors programs to verify runtime by checking values in shared memory.
Tell masterprocess which ones have failed.
"""
import sys
import time
from multiprocessing import shared_memory as shm


SENTINEL_DELAY_PERIOD = 5


def read_cfg() -> list:
    """Read startup integer and load shared memory for relevant programs.
    """
    pass


def watchdog() -> None:
    """Main watchdog function. 
    Validate programs are running that should be.
    Restart any program not running by telling masterprocess.
    """
    sys.stdout.write("Watchdog program started.")


if __name__ == '__main__':
    watchdog()
