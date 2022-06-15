"""Camera client on GUI end.
"""
import cv2
import socket
import pickle
import struct


def run_camera_client(server_ip: str, port: int, test=False) -> None:
    """Run the camera client for the GUI by connecting to the relevant ports.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((server_ip, port))
    s.listen(5)
    conn, addr = s.accept()
    data = b''
    payload_size = struct.calcsize(">L")
    while True:
        while len(data) < payload_size:
            data += conn.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        while len(data) < msg_size:
            data += conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        # Display frames
        if test:
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == ord('q'):
                break


if __name__ == '__main__':
    run_camera_client(server_ip='127.0.0.1', port=50001, test=True)
