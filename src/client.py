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
    files = os.listdir()
    z = 0
    Max = (len(files))
    while z < Max:
        if 'Save' in files[z]:
            directoryPath = os.path.join(os.getcwd(), files[z])
            shutil.rmtree(directoryPath)
        z += 1


def CreateFile():
    parent = os.getcwd()
    directory = 'Save'
    Rpath = os.path.join(parent, directory)
    os.mkdir(Rpath)


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
