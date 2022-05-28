"""
Client reads frames from camera and creates folder in directory for frames to be saved.
"""
import sys
import cv2
import socket
import struct
import pickle
import os
import shutil


def search_file() -> None:
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


def main(host: str, port: int, cap: int, write_frame: bool) -> None:
    """Runs the client code to connect to server and capture frames.
    """
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    cam = cv2.VideoCapture(cap)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    img_counter = 0

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    i = 0
    # Keep receiving frames and save to folder
    while True:
        ret, frame = cam.read()
        # Frame flip for scion
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        # Write to I/O
        if write_frame:
            cv2.imwrite(os.path.join(os.getcwd(), 'vision/') + str(i) + '.jpg', frame)
        i += 1
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame, 0)
        size = len(data)
        print("{}: {}".format(img_counter, size))
        client_socket.sendall(struct.pack(">L", size) + data)
        img_counter += 1


if __name__ == '__main__':
    print('Starting Video Client...')
    if len(sys.argv) > 1:  # Did we get a host?
        hostname = sys.argv[1].replace(' ', '')
    else:
        print(f'Error: Expected argc > 2, number of args = {len(sys.argv)}. (Did you add the server\'s IP address?)')
        print('Exiting Video Client...')
        sys.exit(1)
    search_file()
    main(host=hostname, port=50001, cap=0, write_frame=True)
else:
    print('Scion Video Client imported.')
