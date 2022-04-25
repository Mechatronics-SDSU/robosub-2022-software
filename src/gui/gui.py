"""
Scion's GUI Application (name is pending)

Layout:
(Not to scale)
-------------------------------------------------
Top Bar                    |          | Sensors |
---------------------------- Camera 1 |         |
Buttons | Status | Weapons |----------|         |
---------------------------- Camera 2 |         |
Logging                    |          |         |
-------------------------------------------------


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
edge_size = 2
# Resolution height = top bar + video1 + edge + video2
gui_resolution = (1600, 960)
camera_resolution = (640, 480)  # Both cameras will use 480p
top_bar_resolution = (640, 30)
button_display_resolution = (100, 240)
weapons_status_resolution = (270, 240)
thruster_status_resolution = (270, 240)
logging_resolution = (640, 690)
sensor_resolution = (320, 960)

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
        self.top_bar = tk.Frame(master=self.tk_master, width=top_bar_resolution[0], height=top_bar_resolution[1], bg='black')
        self.button_display = tk.Frame(master=self.tk_master, width=button_display_resolution[0], height=button_display_resolution[1], bg='blue')
        self.weapons_status = tk.Frame(master=self.tk_master, width=weapons_status_resolution[0], height=weapons_status_resolution[1], bg='red')
        self.thruster_status = tk.Frame(master=self.tk_master, width=thruster_status_resolution[0], height=thruster_status_resolution[1], bg='green')
        self.logging = tk.Frame(master=self.tk_master, width=logging_resolution[0], height=logging_resolution[1], bg='grey')
        self.camera = tk.Frame(master=self.tk_master, width=camera_resolution[0], height=camera_resolution[1]*2, bg='orange')
        self.sensors = tk.Frame(master=self.tk_master, width=sensor_resolution[0], height=sensor_resolution[1], bg='yellow')
        # Main grid
        self.top_bar.grid(row=0, column=0, columnspan=3)
        self.button_display.grid(row=1, column=0)
        self.weapons_status.grid(row=1, column=1)
        self.thruster_status.grid(row=1, column=2)
        self.logging.grid(row=2, column=0, columnspan=3)
        self.camera.grid(row=0, column=3, rowspan=3)
        self.sensors.grid(row=0, column=4, rowspan=3)

def init_gui(host_os: str) -> None:
    """Starts up GUI window and all related programs
    """
    if host_os == 'nt':  # Windows
        context = mp.get_context('spawn')
    else:  # Linux
        context = mp.get_context('fork')
    
    # Start GUI
    gui_window = tk.Tk()
    gui_window.geometry(str(gui_resolution[0]) + "x" + str(gui_resolution[1]))
    gui_application = GuiWindow(gui_window)
    gui_window.mainloop()


if __name__ == '__main__':
    if os.name == 'nt':  # Windows
        mp.set_start_method('spawn')
    else:  # Linux
        mp.set_start_method('fork')
    print(f'{__name__} started on {os.name} at {os.getpid()}')  # Kept for testing
    init_gui(host_os=os.name)
