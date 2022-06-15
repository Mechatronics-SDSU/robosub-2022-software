"""Camera client on GUI end.
"""
import cv2
import socket
import pickle
import struct
import multiprocessing


def run_camera_client(server_ip: str, port: int, test=False, write_pipe=None, camera_num=0) -> None:
    """Run the camera client for the GUI by connecting to the relevant ports.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((server_ip, port))
    data = b''
    payload_size = struct.calcsize(">L")
    while True:
        while len(data) < payload_size:
            data += s.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        while len(data) < msg_size:
            data += s.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        # Display frames
        if test:
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == ord('q'):
                break
        else:  # Send the GUI the frame
            write_pipe.send(frame)


if __name__ == '__main__':
    run_camera_client(server_ip='127.0.0.1', port=50001, test=True)
