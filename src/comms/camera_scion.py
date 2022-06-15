"""Scion end of the video driver.
Tests for cameras using opencv, Instantiates as many cameras as possible.
"""
import cv2
import socket
import struct
import pickle
import os
import shutil
import threading


class CamThead(threading.Thread):
    def __init__(self, port_num: int, cap_num: int, write_frame: bool):
        threading.Thread.__init__(self)
        self.port = port_num
        self.cap = cap_num
        self.write_frame = write_frame

    def run(self) -> None:
        video_server(port=self.port, cap=self.cap, write_frame=self.write_frame)


def search_file() -> None:
    """Searches for a folder named "vision" in directory, if it exist the method (shutil.rmtree())
    will delete existing folder along with its contents.
    """
    files = os.listdir()
    z = 0
    max_len = (len(files))
    while z < max_len:
        if 'vision' in files[z]:
            dir_path = os.path.join(os.getcwd(), files[z])
            shutil.rmtree(dir_path)
            break
        z += 1

    # Creates new Save folder for frames to be stored
    parent = os.getcwd()
    directory = 'vision'
    path = os.path.join(parent, directory)
    os.mkdir(path)


def load_cameras() -> list:
    """Attempt to test the video cameras using opencv's builtins.
    """
    functional_ports = []
    dev_max = 199
    dev = 0
    while dev < dev_max:
        cam = cv2.VideoCapture(dev)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if cam.isOpened():  # Camera exists on port
            read, img = cam.read()
            if read:  # Camera works and gets images
                functional_ports.append(dev)
        cam.release()
        dev += 1
    return functional_ports


def video_server(port: int, cap: int, write_frame: bool) -> None:
    """Runs the video server and waits for client connection.
    """
    # CV
    cam = cv2.VideoCapture(cap)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    img_counter = 0
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    # Socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.bind(('', port))
        server_sock.listen(5)
        conn, addr = server_sock.accept()
        while True:
            _, frame = cam.read()
            if _:
                frame = cv2.flip(frame, 0)  # Scion frame flip
                frame = cv2.flip(frame, 1)
                # Write to I/O
                if write_frame:
                    cv2.imwrite(os.path.join(os.getcwd(), 'vision/') + str(img_counter) + '.jpg', frame)
                result, frame = cv2.imencode('.jpg', frame, encode_param)
                data = pickle.dumps(frame, 0)
                size = len(data)
                conn.sendall(struct.pack(">L", size) + data)
                img_counter += 1


def start_video_servers(cameras: list) -> None:
    """Run the video server using the cameras enumerated from the test_cameras function.
    """
    threads = []
    for camera in range(len(cameras)):
        threads.append(CamThead(port_num=int(f'5000{camera+1}'), cap_num=cameras[camera], write_frame=False))
    for thread in threads:
        thread.start()


if __name__ == '__main__':
    cams = load_cameras()
    start_video_servers(cameras=cams)
