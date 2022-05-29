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
3/26/22 on Linux - IAR
3/26/22 on Windows - IAR
"""

from __future__ import print_function
import multiprocessing as mp
from multiprocessing import shared_memory as shm
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
from tkinter import *

import grpc
import pygame as pg
from PIL import ImageTk
from PIL import Image as PILImage
import numpy as np
import cv2

import comms.video_server as scion_vs

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

# Network
SCION_CAMERA_0_PORT = 50001
SCION_CAMERA_1_PORT = 50002
SCION_CONTROL_PORT = 50004
SCION_SENSOR_PORT = 50003
SCION_LOGGING_PORT = 50005


class GuiWindow(tk.Frame):
    """GUI management class
    Contains methods related to GUI functionality; calls to other functions in this file.
    """
    def __init__(self, master, camera_0_pipe: mp.Pipe, camera_1_pipe: mp.Pipe, host_context: mp.context):
        self.tk_master = master
        self.context = host_context
        self.resolution = gui_resolution
        tk.Frame.__init__(self, self.tk_master)
        self.top_bar_fr = tk.Frame(master=self.tk_master, width=top_bar_resolution[0], height=top_bar_resolution[1],
                                   bg='black')
        self.buttons_fr = tk.Frame(master=self.tk_master, width=button_display_resolution[0],
                                   height=button_display_resolution[1], bg='blue')
        self.buttons_cv = tk.Canvas(master=self.buttons_fr, width=button_display_resolution[0],
                                    height=button_display_resolution[1], bg='blue')
        self.buttons_cv_img = ImageTk.PhotoImage(PILImage.open('img/buttons_22.png'))
        self.buttons_cv.create_image((edge_size, edge_size), anchor=tk.NW, image=self.buttons_cv_img)
        self.weapons_fr = tk.Frame(master=self.tk_master, width=weapons_status_resolution[0],
                                   height=weapons_status_resolution[1], bg='red')
        self.thrusters_fr = tk.Frame(master=self.tk_master, width=thruster_status_resolution[0],
                                     height=thruster_status_resolution[1], bg='green')
        self.logging_fr = tk.Frame(master=self.tk_master, width=logging_resolution[0], height=logging_resolution[1],
                                   bg='grey')
        # Cameras
        self.cameras_fr = tk.Frame(master=self.tk_master, width=camera_resolution[0], height=camera_resolution[1] * 2,
                                   bg='orange')
        self.camera_0_cv = tk.Canvas(master=self.cameras_fr, width=camera_resolution[0], height=camera_resolution[1],
                                     bg='orange')
        self.camera_1_cv = tk.Canvas(master=self.cameras_fr, width=camera_resolution[0], height=camera_resolution[1],
                                     bg='orange')
        self.camera_0_img_1 = ImageTk.PhotoImage(PILImage.open('img/not_loaded_2.png'))
        self.camera_0_img_2 = ImageTk.PhotoImage(PILImage.open('img/not_loaded_2.png'))
        self.camera_1_img_1 = ImageTk.PhotoImage(PILImage.open('img/not_loaded_2.png'))
        self.camera_1_img_2 = ImageTk.PhotoImage(PILImage.open('img/not_loaded_2.png'))
        self.video_no_img_itk = ImageTk.PhotoImage(PILImage.open('img/not_loaded.png'))
        self.video_window_img_cv_0 = self.camera_0_cv.create_image((1, 1), anchor=tk.NW, image=self.video_no_img_itk)
        self.video_window_img_cv_1 = self.camera_1_cv.create_image((1, 1), anchor=tk.NW, image=self.video_no_img_itk)
        self.camera_0_shm = shm.SharedMemory('video_server_0_shm')
        self.camera_0_pipe = camera_0_pipe
        self.camera_0_frame_counter = 0
        self.camera_1_shm = shm.SharedMemory('video_server_1_shm')
        self.camera_1_pipe = camera_1_pipe
        self.camera_1_frame_counter = 0
        self.sensors_fr = tk.Frame(master=self.tk_master, width=sensor_resolution[0], height=sensor_resolution[1],
                                   bg='yellow')
        # Main grid
        self.top_bar_fr.grid(row=0, column=0, columnspan=3)
        self.buttons_fr.grid(row=1, column=0)
        self.weapons_fr.grid(row=1, column=1)
        self.thrusters_fr.grid(row=1, column=2)
        self.logging_fr.grid(row=2, column=0, columnspan=3)
        self.cameras_fr.grid(row=0, column=3, rowspan=3)
        self.sensors_fr.grid(row=0, column=4, rowspan=3)

        # Sub-grids
        # Button Grid
        self.buttons_cv.grid()
        self.cam_0_button = Button(master=self.buttons_cv, text='Start Cam 0', justify=LEFT, anchor='w',
                                   command=self.start_camera_0)
        self.cam_1_button = Button(master=self.buttons_cv, text='Start Cam 1', justify=LEFT, anchor='w',
                                   command=self.start_camera_1)
        self.cam_0_button.grid(row=0, column=0, sticky=W)
        self.cam_1_button.grid(row=1, column=0, sticky=W)
        # Camera Grid
        self.camera_0_cv.grid(row=0, column=0)
        self.camera_1_cv.grid(row=1, column=0)

        self.update()

    def start_camera_0(self) -> None:
        self.camera_0_shm.buf[0] = 1

    def start_camera_1(self) -> None:
        self.camera_1_shm.buf[0] = 1

    def update_sensors(self) -> None:
        """Update sensor data in the tkinter window by reading the sensor thread.
        Get data stored in sensor thread and calculate ETA since obtained.
        """
        pass

    def update_cameras(self) -> None:
        """Update camera frames in the tkinter window by reading the camera process.
        Newest frame in the camera thread is loaded into either pillow frame 1 or 2 depending on what frame counter we
        are at. The new frame is checked with a modulus 2 to see which pillow frame to display in the window.
        """
        if self.camera_0_shm.buf[0] == 2:  # Camera 0 has frames
            conn_0 = mp.connection.wait([self.camera_0_pipe], timeout=-1)
            if len(conn_0) > 0:  # Frame in pipe
                frame_0 = conn_0[0].recv()
                # Convert from opencv BGR to Pillow RGB
                b, g, r = cv2.split(frame_0)
                image_0 = cv2.merge((r, g, b))
                # Alternate between frames. Calling ImageTk then video_window itemconfig garbage collects the old frame
                self.camera_0_frame_counter += 1
                # Set the frame to pillow canvas
                if self.camera_0_frame_counter % 2 == 1:
                    self.camera_0_img_2 = ImageTk.PhotoImage(PILImage.fromarray(image_0))
                    self.camera_0_cv.itemconfig(self.video_window_img_cv_0, image=self.camera_0_img_2)
                else:
                    self.camera_0_img_1 = ImageTk.PhotoImage(PILImage.fromarray(image_0))
                    self.camera_0_cv.itemconfig(self.video_window_img_cv_0, image=self.camera_0_img_1)

        if self.camera_1_shm.buf[0] == 2:  # Camera 1 has frames
            conn_1 = mp.connection.wait([self.camera_1_pipe], timeout=-1)
            if len(conn_1) > 0:  # Frame in pipe
                frame_1 = conn_1[0].recv()
                # Convert from opencv BGR to Pillow RGB
                b, g, r = cv2.split(frame_1)
                image_1 = cv2.merge((r, g, b))
                # Alternate between frames. Calling ImageTk then video_window itemconfig garbage collects the old frame
                self.camera_1_frame_counter += 1
                # Set the frame to pillow canvas
                if self.camera_1_frame_counter % 2 == 1:
                    self.camera_1_img_2 = ImageTk.PhotoImage(PILImage.fromarray(image_1))
                    self.camera_1_cv.itemconfig(self.video_window_img_cv_1, image=self.camera_1_img_2)
                else:
                    self.camera_1_img_1 = ImageTk.PhotoImage(PILImage.fromarray(image_1))
                    self.camera_1_cv.itemconfig(self.video_window_img_cv_1, image=self.camera_1_img_1)

    def update(self) -> None:
        """Update dynamic elements in the GUI window, read elements from other threads.
        Overridden from tkinter's window class
        """
        self.update_cameras()

        self.after(gui_update_ms, self.update)  # Run this function again after delay of gui_update_ms


def run_video_client(wvc: mp.Pipe, server_port: int, start_context: mp.context, camera_num: int) -> None:
    """Run the imported video server from comms, passing the pipe as an argument
    """
    scion_vs.main(host='', port=server_port, ind=False, write_pipe=wvc, context=start_context, camera_num=camera_num)


def init_gui(host_context: mp.context) -> None:
    """Starts up GUI window and all related programs
    """
    # Set up UNIX Pipes for communication between processes
    # w = write end, r = read end
    wvs_pipe_0, rvs_pipe_0 = host_context.Pipe()  # Camera 0 (write end) -> | -> GUI (read end)
    wvs_pipe_1, rvs_pipe_1 = host_context.Pipe()  # Camera 1 (write end) -> | -> GUI (read end)

    # Shared start integers
    try:  # Validate shared memory was closed properly
        shm_vs_0 = shm.SharedMemory(create=True, size=1, name='video_server_0_shm')
    except FileExistsError:
        shm_vs_0 = shm.SharedMemory(name='video_server_0_shm')
        shm_vs_0.unlink()
        shm_vs_0 = shm.SharedMemory(create=True, size=1, name='video_server_0_shm')
    shm_vs_0.buf[0] = 0
    try:  # Validate shared memory was closed properly
        shm_vs_1 = shm.SharedMemory(create=True, size=1, name='video_server_1_shm')
    except FileExistsError:
        shm_vs_1 = shm.SharedMemory(name='video_server_1_shm')
        shm_vs_1.unlink()
        shm_vs_1 = shm.SharedMemory(create=True, size=1, name='video_server_1_shm')
    shm_vs_1.buf[0] = 0
    # Start video server(s)
    camera_0_proc = host_context.Process(target=run_video_client, args=(wvs_pipe_0, SCION_CAMERA_0_PORT, host_context,
                                                                        0))
    camera_1_proc = host_context.Process(target=run_video_client, args=(wvs_pipe_1, SCION_CAMERA_1_PORT, host_context,
                                                                        1))

    camera_0_proc.start()
    camera_1_proc.start()

    # Start GUI
    gui_window = tk.Tk()
    gui_window.geometry(str(gui_resolution[0]) + "x" + str(gui_resolution[1]))
    gui_application = GuiWindow(gui_window, camera_0_pipe=rvs_pipe_0, camera_1_pipe=rvs_pipe_1,
                                host_context=host_context)
    gui_window.mainloop()


if __name__ == '__main__':
    if os.name == 'nt':  # Windows
        context = mp.get_context('spawn')
        print('GUI not supported on Windows')
        system.exit(1)
    else:  # Linux
        mp.set_start_method('fork')
        context = mp.get_context('fork')
    print(f'GUI started on {os.name} at {os.getpid()}')  # Kept for testing
    init_gui(host_context=context)
else:
    print(f'Child started at {os.getpid()}')
