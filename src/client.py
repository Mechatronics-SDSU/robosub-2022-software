"""
Client reads frames from camera and creates folder in directory for frames to be saved
"""
import cv2
import io
import socket
import struct
import time
import pickle
import zlib
import os
import shutil


def SearchFile():
    """ 
    searches for a folder named "Save" in directory, if it exist the method (shutil.rmtree()) will delete 
    existing folder along with its contents.
    """
    files = os.listdir()
    z = 0
    Max = (len(files))
    while z < Max:
        if 'Save' in files[z]:
            directoryPath = os.path.join(os.getcwd(), files[z])
            shutil.rmtree(directoryPath)
        z += 1


def CreateFile():
    """ 
    Creates new Save folder for frames to be stored
    """
    parent = os.getcwd()
    directory = 'Save'
    Rpath = os.path.join(parent, directory)
    os.mkdir(Rpath)

#connection to server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('localhost', 1234))
connection = client_socket.makefile('wb')
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
img_counter = 0

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
SearchFile()
CreateFile()
i = 0

#while true, keep recieving frames and save to folder
while True:
    ret, frame = cam.read()
    cv2.imwrite(os.path.join(os.getcwd(), 'Save/') + str(i) + '.jpg', frame)
    i += 1
    result, frame = cv2.imencode('.jpg', frame, encode_param)
    #    data = zlib.compress(pickle.dumps(frame, 0))
    data = pickle.dumps(frame, 0)
    size = len(data)
    print("{}: {}".format(img_counter, size))
    client_socket.sendall(struct.pack(">L", size) + data)
    img_counter += 1

cam.release()
