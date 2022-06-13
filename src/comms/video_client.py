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
import threading


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


class CamThead(threading.Thread):
    def __init__(self, host: str, port_num: int, cap_num: int, write_frame: bool):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port_num
        self.cap = cap_num
        self.write_frame = write_frame

    def run(self) -> None:
        main(host=self.host, port=self.port, cap=self.cap, write_frame=self.write_frame)


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
        if ret:
            # Frame flip for scion
            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)
            # Write to I/O
            if write_frame:
                cv2.imwrite(os.path.join(os.getcwd(), 'vision/') + str(img_counter) + '.jpg', frame)
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            data = pickle.dumps(frame, 0)
            size = len(data)
            print("{}: {}".format(img_counter, size))
            client_socket.sendall(struct.pack(">L", size) + data)
            img_counter += 1


if __name__ == '__main__':
    print('Starting Video Client...')
    if len(sys.argv) > 3:  # Did we get a host/camera number?
        hostname = sys.argv[1].replace(' ', '')
        _port = int(sys.argv[2].replace(' ', ''))
        _cap = int(sys.argv[3].replace(' ', ''))
        if len(sys.argv) > 5:
            _port_2 = int(sys.argv[4].replace(' ', ''))
            _cap_2 = int(sys.argv[5].replace(' ', ''))
            search_file()
            t1 = CamThead(host=hostname, port_num=_port, cap_num=_cap, write_frame=False)
            t2 = CamThead(host=hostname, port_num=_port_2, cap_num=_cap_2, write_frame=False)
            t1.start()
            t2.start()
        else:
            search_file()
            main(host=hostname, port=_port, cap=_cap, write_frame=False)
    else:
        print(f'Error: Expected argc > 2, number of args = {len(sys.argv)}. (Did you add the server\'s IP address?)')
        print('Exiting Video Client...')
        sys.exit(1)

else:
    print('Scion Video Client imported.')
