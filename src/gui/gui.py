"""
Scion's GUI Application (name is pending)

Latest verification test:
3/16/22 on Linux - IAR
3/17/22 on Windows - IAR
"""

from __future__ import print_function
import multiprocessing as mp
import os
import sys as system
import math
from collections import deque
import socket
from datetime import datetime
import time
from functools import partial
import pickle
import struct

import tkinter as tk


import grpc
import pygame as pg
from PIL import ImageTk
from PIL import Image as PILImage
import numpy as np
import cv2

# GUI constants
top_bar_height = 30
edge_size = 2
# Resolution height = top bar + video1 + edge + video2
gui_resolution = (1600, 960)
camera_resolution = (640, 480)
gui_update_ms = 10  # Update time for all GUI elements 
color_term_green = (74, 246, 38)
color_error_red = (255, 0, 3)


class GuiWindow(tk.Frame):
    """GUI management class
    Contains methods related to GUI functionality; calls to other functions in this file.
    """
    def __init__(self, master):
        self.tk_master = master
        self.resolution = gui_resolution
        tk.Frame.__init__(self, self.tk_master)
        self.top_bar = tk.Frame(master=self.tk_master, width=self.resolution[0], height=top_bar_height, bg='white')


def init_gui() -> None:
    """Starts up GUI window and all related programs
    """
    if os.name == 'nt':  # Windows
        context = mp.get_context('spawn')
    else:  # Linux
        context = mp.get_context('fork')
    
    # Start GUI
    gui_window = tk.Tk()
    gui_window.geometry(str(gui_resolution[0]) + "x" + str(gui_resolution[1]))
    gui_application = GuiWindow(gui_window)
    gui_window.mainloop()


if __name__ == '__main__':
    n = os.name
    if n == 'nt':  # Windows
        mp.set_start_method('spawn')
    else:  # Linux
        mp.set_start_method('fork')
    print(f'{__name__} started on {n} at {os.getpid()}')  # Kept for testing
    init_gui()
