"""Camera client on GUI end.
"""
import cv2
import socket
import pickle
import struct
import queue as q
import multiprocessing


def run_camera_client(write_pipe: multiprocessing.Queue, server_ip: str, port: int, test=False, camera_num=0,
                      context=multiprocessing.context) -> None:
    """Run the camera client for the GUI by connecting to the relevant ports.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reuse address to prevent errno 98
    s.connect((server_ip, port))
    data = b''
    frame_count = 0
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
        # print(f'[CAM] LOADED frame with data of size: {len(frame_data)} and msg_size: {msg_size}')
        frame_count += 1
        # Display frames
        if test:
            frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == ord('q'):
                break
        else:  # Send the GUI the frame
            try:
                write_pipe.put_nowait(frame_data)
            except q.Full:
                # print('QUEUE IS FULL!')

            # Old pipe data
            # write_pipe.send(frame_data)
            # print(f'[CAM] Frame count: {frame_count}')


if __name__ == '__main__':
    ip = '127.0.0.1'
    print(f'Starting camera client on {ip}')
    run_camera_client(server_ip=ip, port=50001, test=True)
