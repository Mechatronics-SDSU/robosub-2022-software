"""
Client reads frames from camera and creates folder in directory for frames to be saved.
Meant to be run as the main file, not being imported.
"""
import cv2
import socket
import struct
import pickle
import os
import shutil

def SearchFile():
    """
    Searches for a folder named "vision" in directory, if it exist the method (shutil.rmtree())
    will delete existing folder along with its contents.
    """
    files = os.listdir()
    z = 0
    Max = (len(files))
    while z < Max:
        if 'vision' in files[z]:
            directoryPath = os.path.join(os.getcwd(), files[z])
            shutil.rmtree(directoryPath)
            break
        z += 1
    
    # Creates new Save folder for frames to be stored
    parent = os.getcwd()
    directory = 'vision'
    Rpath = os.path.join(parent, directory)
    os.mkdir(Rpath)

def main() -> None:
    """Runs the client code to connect to server and capture frames.
    """

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('localhost', 1234))
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    img_counter = 0

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    i = 0

    # Keep receiving frames and save to folder
    while True:
        ret, frame = cam.read()
        cv2.imwrite(os.path.join(os.getcwd(), 'vision/') + str(i) + '.jpg', frame)
        i += 1
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame, 0)
        size = len(data)
        print("{}: {}".format(img_counter, size))
        client_socket.sendall(struct.pack(">L", size) + data)
        img_counter += 1

    cam.release()


if __name__ == '__main__':
    SearchFile()
    main()