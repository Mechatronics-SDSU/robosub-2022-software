"""
Video Server connects to Video Client and displays read frames in window.
"""
import sys
import socket
import cv2
import pickle
import struct
import time
import multiprocessing as mp
from multiprocessing import shared_memory as shm

thread_sleep_s = 0.25


def main(host: str, port: int, ind: bool, write_pipe=None, context=None, camera_num=0) -> None:
    """Runs server code for our client to have a connection, server receives and display frames
    """
    HOST = host
    PORT = port
    shm_camera = shm.SharedMemory(name=f'video_server_{camera_num}_shm')
    while shm_camera.buf[0] == 0:  # Wait for GUI to signal ready
        time.sleep(thread_sleep_s)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f'Video socket created on port {PORT}')
    s.bind((HOST, PORT))
    s.listen(5)
    print(f'Video socket bound, listening on port {PORT}')
    conn, addr = s.accept()
    data = b""
    payload_size = struct.calcsize(">L")
    print("payload_size: {}".format(payload_size))
    # Display frames and print
    while True:
        while len(data) < payload_size:
            # print("Recv: {}".format(len(data)))
            data += conn.recv(4096)
        if shm_camera.buf[0] == 1:
            shm_camera.buf[0] = 2  # We're now receiving frames
        # print("Done Recv: {}".format(len(data)))
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        # print("msg_size: {}".format(msg_size))
        while len(data) < msg_size:
            data += conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        # Frames displayed
        if ind:  # Running on its own
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == ord('q'):
                break
        else:  # Running in GUI
            write_pipe.send(frame)  # Send to GUI


if __name__ == '__main__':
    print('Starting Video Server...')
    main('', 50001, ind=True)
else:
    print('Scion Video Server imported.')
