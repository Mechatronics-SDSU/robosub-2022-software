"""
Scion's GUI Application (name is pending)
This is mostly a port of Pico's GUI with some changes:
0. Well thought out algorithm and design decisons
1. Using a sensible way of sharing memory (multiprocessing.SharedMemory )that isn't using UNIX pipes.
2. Support for 2 cameras instead of 1
3. Support for far more flexible APIs

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
6/4/22 on Linux - IAR
"""

from __future__ import print_function
import multiprocessing as mp
import sys
from multiprocessing import shared_memory as shm
import os
from os.path import exists
import sys as system
# import math
# from collections import deque
import socket
# from datetime import datetime
import time
from functools import partial
# import pickle
import struct

import tkinter as tk
from tkinter import *
# from tkinter import simpledialog

import pygame as pg
from PIL import ImageTk
from PIL import Image as PILImage
# import numpy as np
import cv2

import gen_default_text as scion_gdt
import comms.cmd_ctrl_client as scion_cnc
import comms.camera_gui as scion_cam
import comms.controller_client as scion_cc
import sensor.telemetry_linker as scion_tl
import utils.scion_utils as scion_ut

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
sleep_thread_s = 0.25  # Time to check for new threads to be enabled
color_term_green = (74, 246, 38)
color_error_red = (255, 0, 3)

# Network
SCION_DEFAULT_IPV4 = '192.168.3.1'
SCION_COMMAND_PORT = 50000
SCION_CAMERA_0_PORT = 50001
SCION_CAMERA_1_PORT = 50002
SCION_CONTROL_PORT = 50004
SCION_SENSOR_PORT = 50003
SCION_LOGGING_PORT = 50005

# Master process
SCION_CONFIG_MENU_STRINGS = [  # Enumerated for consistency with start/masterprocess.h
    'AIO',
    'Sensor API',
    'AHRS Sensor',
    'Depth Sensor',
    'Thrusters',
    'Cameras'
]


class GuiWindow(tk.Frame):
    """GUI management class
    Contains methods related to GUI functionality; calls to other functions in this file.
    """
    def __init__(self, master, camera_0_pipe: mp.Pipe, camera_1_pipe: mp.Pipe, logging_pipe: mp.Pipe,
                 host_context: mp.context):
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
        # Ports
        self.def_ports = (SCION_COMMAND_PORT, SCION_CAMERA_0_PORT, SCION_CAMERA_1_PORT, SCION_SENSOR_PORT,
                          SCION_CONTROL_PORT, SCION_LOGGING_PORT)
        self.current_ports = [SCION_COMMAND_PORT, SCION_CAMERA_0_PORT, SCION_CAMERA_1_PORT, SCION_SENSOR_PORT,
                              SCION_CONTROL_PORT, SCION_LOGGING_PORT]
        self.cmd_port = self.current_ports[0]
        self.camera_0_port = self.current_ports[1]
        self.camera_1_port = self.current_ports[2]
        self.telemetry_port = self.current_ports[3]
        self.pilot_port = self.current_ports[4]
        self.logging_port = self.current_ports[5]

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

        # Sensors/Telemetry
        sensor_exists = exists('img/sensor_22.png')
        if not sensor_exists:  # Generate new sensor image based on config in default text gen
            scion_gdt.generate_sensor_comms_window()
        self.sensors_fr = tk.Frame(master=self.tk_master, width=sensor_resolution[0], height=sensor_resolution[1],
                                   bg='yellow')
        self.telemetry_canvas_0 = tk.Canvas(master=self.sensors_fr, width=sensor_resolution[0],
                                            height=sensor_resolution[1], bd=0, bg='green')
        self.telemetry_canvas_0_img = ImageTk.PhotoImage(PILImage.open('img/sensor_22.png'))
        self.telemetry_canvas_1_img = ImageTk.PhotoImage(PILImage.open('img/sensor_22.png'))
        self.telemetry_frame_counter = 0
        self.telemetry_canvas_0_config = self.telemetry_canvas_0.create_image((1, 1),
                                                                              anchor=tk.NW,
                                                                              image=self.telemetry_canvas_0_img)
        self.sensors_cv = tk.Canvas(master=self.master, width=sensor_resolution[0], height=sensor_resolution[1],
                                    bg='black')
        self.telemetry_ctrl_shm = shm.SharedMemory(name='telemetry_ctrl_shm')
        self.telemetry_linker = scion_tl.TelemetryLinker(use_shm=True)

        # Pilot
        self.pilot_ctrl_shm = shm.SharedMemory(name='pilot_ctrl_shm')

        # Logging
        self.logging_pipe = logging_pipe
        self.logging_ctrl_shm = shm.SharedMemory(name='logging_ctrl_shm')

        self.camera_0_enable = tk.BooleanVar(value=False)
        self.camera_1_enable = tk.BooleanVar(value=False)
        self.telemetry_enable = tk.BooleanVar(value=False)
        self.pilot_enable = tk.BooleanVar(value=False)
        self.logging_enable = tk.BooleanVar(value=False)

        # Master process
        self.masterprocess = []
        for i in range(len(SCION_CONFIG_MENU_STRINGS)):  # Make a tkinter boolean for everything in master process
            self.masterprocess.append(tk.BooleanVar(value=False))
        self.cnc_shm = shm.SharedMemory(name='cnc_shm')
        self.cnc_val_shm = shm.SharedMemory(name='cnc_val_shm')

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
        self.config_button = Button(master=self.top_bar_fr, text='Set Masterprocess', justify=LEFT, anchor='w',
                                    command=self.set_config_menu)
        self.conn_button = Button(master=self.top_bar_fr, text='Connect', justify=LEFT, anchor='w',
                                  command=self.send_masterprocess)
        self.cam_0_button = Button(master=self.top_bar_fr, text='Start Cam 0', justify=LEFT, anchor='w',
                                   command=self.start_camera_0)
        self.cam_1_button = Button(master=self.top_bar_fr, text='Start Cam 1', justify=LEFT, anchor='w',
                                   command=self.start_camera_1)
        self.tel_button = Button(master=self.top_bar_fr, text='Start Sensors', justify=LEFT, anchor='w',
                                 command=self.start_telemetry)
        self.plt_button = Button(master=self.top_bar_fr, text='Start Pilot', justify=LEFT, anchor='w',
                                 command=self.start_pilot)
        self.config_button.grid(row=0, column=0, sticky=W)
        self.conn_button.grid(row=0, column=1, sticky=W)
        self.cam_0_button.grid(row=0, column=2, sticky=W)
        self.cam_1_button.grid(row=0, column=3, sticky=W)
        self.tel_button.grid(row=0, column=4, sticky=W)
        self.plt_button.grid(row=0, column=5, sticky=W)
        # Camera Grid
        self.camera_0_cv.grid(row=0, column=0)
        self.camera_1_cv.grid(row=1, column=0)
        # Sensors
        self.telemetry_canvas_0.grid()

        # Turn on cnc
        self.cnc_shm.buf[0] = 1

        self.update()

    def val_set(self, old: any, new: any) -> None:
        """tkinter doesn't like calling old.set() within command= arguments, so it's done here
        """
        old.set(new)
        self.update_host_display()

    def set_config_menu(self) -> None:
        """Set the config menu for what to enable, then send command packet to Scion.
        """
        # Build labels for boxes
        top = tk.Toplevel(self.master)  # Put this window on top
        config_lb = tk.Label(top, text='Enable Programs', pady=10, justify='left', anchor='nw')
        config_lb.grid(column=0, row=0, sticky=W, columnspan=2)
        mp_lbs = []
        mp_diags = []
        for i in range(len(SCION_CONFIG_MENU_STRINGS)):
            mp_lbs.append(tk.Label(top, text=SCION_CONFIG_MENU_STRINGS[i]))
            mp_diags.append(tk.Label(top))
            Radiobutton(mp_diags[i], text='Enable', variable=self.masterprocess[i], value=1,
                        command=partial(self.val_set, self.masterprocess[i], True)).grid(column=0, row=0)
            Radiobutton(mp_diags[i], text='Disable', variable=self.masterprocess[i], value=0,
                        command=partial(self.val_set, self.masterprocess[i], False)).grid(column=1, row=0)
            mp_lbs[i].grid(column=0, row=i+1, sticky=W)
            mp_diags[i].grid(column=1, row=i+1, sticky=W)

    def send_masterprocess(self) -> None:
        """Sets the masterprocess configuration in shm to send.
        """
        # Serialize booleans to calculate s argument to masterprocess
        sarg = 0
        for i in range(len(self.masterprocess)):
            if self.masterprocess[i].get() is True:
                sarg += pow(2, i)
        sarg_struct = struct.pack('>q', sarg)  # Pack for shm
        for i in range(8):
            self.cnc_val_shm.buf[i] = sarg_struct[i]  # Set s argument in shared memory first
        self.cnc_shm.buf[0] = 3  # Then enable CNC process to send configuration

    def start_camera_0(self) -> None:
        self.camera_0_shm.buf[0] = 1

    def start_camera_1(self) -> None:
        self.camera_1_shm.buf[0] = 1

    def start_telemetry(self) -> None:
        self.telemetry_ctrl_shm.buf[0] = 1

    def start_logging(self) -> None:
        self.logging_ctrl_shm.buf[0] = 1

    def start_pilot(self) -> None:
        self.pilot_ctrl_shm.buf[0] = 1

    def update_sensors(self) -> None:
        """Update data in the tkinter window by reading what was last sent on the sensor thread.
        """
        if self.telemetry_ctrl_shm.buf[0] == 2:  # Telemetry has data
            self.telemetry_linker.load_all()  # Get data stored in sensor shm and load it into linker
            # Text elements for displaying data
            sensor_frame = cv2.imread('img/sensor_22.png')
            # Load text into an opencv frame
            cv2.putText(img=sensor_frame, text=f'{round(self.telemetry_linker.data[0], 3)}',
                        org=(150, scion_ut.RIGHT_WIN_CONF[8]), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.2,
                        color=color_term_green, thickness=1)
            cv2.putText(img=sensor_frame, text=f'{round(self.telemetry_linker.data[1], 3)}',
                        org=(150, scion_ut.RIGHT_WIN_CONF[9]), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.2,
                        color=color_term_green, thickness=1)
            cv2.putText(img=sensor_frame, text=f'{round(self.telemetry_linker.data[2], 3)}',
                        org=(150, scion_ut.RIGHT_WIN_CONF[10]), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.2,
                        color=color_term_green, thickness=1)
            cv2.putText(img=sensor_frame, text=f'{round(self.telemetry_linker.data[3], 3)}',
                        org=(150, scion_ut.RIGHT_WIN_CONF[11]), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.2,
                        color=color_term_green, thickness=1)
            # Convert to be tkinter compatible
            self.telemetry_frame_counter += 1
            if self.telemetry_frame_counter % 2 == 1:
                self.telemetry_canvas_1_img = ImageTk.PhotoImage(PILImage.fromarray(sensor_frame))
                self.telemetry_canvas_0.itemconfig(self.telemetry_canvas_0_config, image=self.telemetry_canvas_1_img)
            else:
                self.telemetry_canvas_0_img = ImageTk.PhotoImage(PILImage.fromarray(sensor_frame))
                self.telemetry_canvas_0.itemconfig(self.telemetry_canvas_0_config, image=self.telemetry_canvas_1_img)
            # print(self.telemetry_linker.data)  # Temporary until we can verify

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

    def update_host_display(self) -> None:
        self.camera_0_shm.buf[0] = int(self.camera_0_enable.get() is True)
        self.camera_1_shm.buf[0] = int(self.camera_1_enable.get() is True)
        self.telemetry_ctrl_shm.buf[0] = int(self.telemetry_enable.get() is True)
        self.pilot_ctrl_shm.buf[0] = int(self.pilot_enable.get() is True)
        self.pilot_ctrl_shm.buf[1] = 4

    def update(self) -> None:
        """Update dynamic elements in the GUI window, read elements from other threads.
        Overridden from tkinter's window class
        """
        self.update_cameras()
        self.update_sensors()
        self.after(gui_update_ms, self.update)  # Run this function again after delay of gui_update_ms


def run_cnc_server() -> None:
    """Run the command and control server for setting Scion's configuration before starting.
    """
    cnc_shm = shm.SharedMemory(name='cnc_shm')
    cnc_val_shm = shm.SharedMemory(name='cnc_val_shm')
    cnc = None
    while True:
        if cnc_shm.buf[0] == 1:  # CNC enabled
            cnc_shm.buf[0] = 2  # Switch to CNC on
            cnc = scion_cnc.CNCWrapper(host=SCION_DEFAULT_IPV4, port=SCION_COMMAND_PORT, debug=False)
        elif cnc_shm.buf[0] == 3:  # GUI has indicated cnc is ready to send, read into a long long
            size_mp = 8
            b = bytearray(size_mp)
            for i in range(size_mp):
                b[i] = cnc_val_shm.buf[i]

            cnc.send_message(b)  # Send mp i64
            cnc_shm.buf[0] = 0  # Turn off CNC server in shm
            sys.exit(0)  # Exit CNC server, it has successfully sent configuration
        time.sleep(0.1)


def run_video_client(wvc: mp.Pipe, server_port: int, start_context: mp.context, camera_num: int) -> None:
    """Run the imported video server from comms, passing the pipe as an argument
    """
    camera_0_shm = shm.SharedMemory(name='video_server_0_shm')
    while True:
        if camera_0_shm.buf[0] == 1:
            camera_0_shm.buf[0] = 2
            print('Running camera client.')
            scion_cam.run_camera_client(server_ip=SCION_DEFAULT_IPV4, port=50001, write_pipe=wvc,
                                        camera_num=camera_num)


def run_telemetry_client(scion_ip: str, server_port: int) -> None:
    """Run the telemetry client for receiving sensor data.
    """
    # Setup shared memory
    telemetry_ctrl_shm = shm.SharedMemory(name='telemetry_ctrl_shm')
    # Setup telemetry linker
    linker = scion_tl.TelemetryLinker(use_shm=True)

    while True:
        if telemetry_ctrl_shm.buf[0] == 0:  # Wait for telemetry to be enabled
            time.sleep(sleep_thread_s)
        else:  # Establish a connection and put transmitted data into shared memory
            print("Starting telemetry socket")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reuse the address to bypass errno 98
                try:
                    s.connect((scion_ip, server_port))
                    telemetry_ctrl_shm.buf[0] = 2
                except ConnectionRefusedError as e:
                    print("Error with connecting to telemetry server.")
                    print(e)
                    telemetry_ctrl_shm.buf[0] = 0
                while telemetry_ctrl_shm.buf[0] == 2:
                    try:
                        s.sendall(b'1')
                    except (ConnectionAbortedError, ConnectionResetError):
                        telemetry_ctrl_shm.buf[0] = 1
                        break
                    try:
                        data = s.recv(4096)
                    except (ConnectionAbortedError, ConnectionResetError):
                        telemetry_ctrl_shm.buf[0] = 1
                        break
                    # Parse into telemetry linker
                    linker.unpack_data(loading_pickle=data)


def run_pilot_client() -> None:
    """Run the pilot client for controlling scion remotely.
    """
    # Setup shm
    pilot_shm = shm.SharedMemory(name='pilot_ctrl_shm')
    # Loop waiting for enable
    while True:
        if pilot_shm.buf[0] == 0:  # Wait for pilot to be enabled
            time.sleep(sleep_thread_s)
        else:  # Validate controller exists before starting up client
            pg.joystick.init()
            if pg.joystick.get_count() > 0:  # Start client
                scion_cc.pilot_proc(argc=3, argv=['', SCION_DEFAULT_IPV4, str(SCION_CONTROL_PORT)])
            else:
                print('ERROR: Attempted to start pilot without joystick')
                pilot_shm.buf[0] = 0


def run_logging_client(out_pipe: mp.Pipe, _: str) -> None:
    """Run the log client, setting up a pipe to the GUI to receive strings.
    """
    pass


def init_gui(host_context: mp.context) -> None:
    """Starts up GUI window and all related programs
    """
    # Set up UNIX Pipes for communication between processes. w = write end, r = read end
    wvs_pipe_0, rvs_pipe_0 = host_context.Pipe()  # Camera 0 (write end) -> | -> GUI (read end)
    wvs_pipe_1, rvs_pipe_1 = host_context.Pipe()  # Camera 1 (write end) -> | -> GUI (read end)

    # Shared start integers
    # CNC
    try:
        cnc_shm = shm.SharedMemory(create=True, size=1, name='cnc_shm')
    except FileExistsError:
        cnc_shm = shm.SharedMemory(name='cnc_shm')
        cnc_shm.unlink()
        cnc_shm = shm.SharedMemory(create=True, size=1, name='cnc_shm')
    cnc_shm.buf[0] = 0
    try:
        cnc_val_shm = shm.SharedMemory(create=True, size=8, name='cnc_val_shm')
    except FileExistsError:
        cnc_val_shm = shm.SharedMemory(name='cnc_val_shm')
        cnc_val_shm.unlink()
        cnc_val_shm = shm.SharedMemory(create=True, size=8, name='cnc_val_shm')
    for i in range(8):
        cnc_val_shm.buf[i] = 0
    cnc_proc = host_context.Process(target=run_cnc_server)
    cnc_proc.start()

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

    # Start telemetry client
    try:
        telemetry_ctrl_shm = shm.SharedMemory(create=True, size=1, name='telemetry_ctrl_shm')
    except FileExistsError:
        telemetry_ctrl_shm = shm.SharedMemory(name='telemetry_ctrl_shm')
        telemetry_ctrl_shm.unlink()
        telemetry_ctrl_shm = shm.SharedMemory(create=True, size=1, name='telemetry_ctrl_shm')
    telemetry_proc = host_context.Process(target=run_telemetry_client, args=(SCION_DEFAULT_IPV4, SCION_SENSOR_PORT))
    telemetry_proc.start()

    # Start pilot client
    try:
        pilot_shm = shm.SharedMemory(create=True, size=2, name='pilot_ctrl_shm')
    except FileExistsError:
        pilot_shm = shm.SharedMemory(name='pilot_ctrl_shm')
        pilot_shm.unlink()
        pilot_shm = shm.SharedMemory(create=True, size=2, name='pilot_ctrl_shm')
    pilot_shm.buf[0] = 0
    pilot_shm.buf[1] = SCION_CONTROL_PORT - SCION_COMMAND_PORT
    pilot_proc = host_context.Process(target=run_pilot_client)
    pilot_proc.start()

    # Start log client
    # Set up UNIX Pipes for communication between processes. w = write end, r = read end
    wls_pipe_0, rls_pipe_0 = host_context.Pipe()  # Logging Process (write end) -> | -> GUI (read end)
    try:
        logging_shm = shm.SharedMemory(create=True, size=1, name='logging_ctrl_shm')
    except FileExistsError:
        logging_shm = shm.SharedMemory(name='logging_ctrl_shm')
        logging_shm.unlink()
        logging_shm = shm.SharedMemory(create=True, size=1, name='logging_ctrl_shm')

    logging_proc = host_context.Process(target=run_logging_client, args=(wls_pipe_0, ''))
    logging_proc.start()

    # Start GUI
    gui_window = tk.Tk()
    gui_window.geometry(str(gui_resolution[0]) + "x" + str(gui_resolution[1]))
    gui_application = GuiWindow(gui_window, camera_0_pipe=rvs_pipe_0, camera_1_pipe=rvs_pipe_1, logging_pipe=rls_pipe_0,
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
