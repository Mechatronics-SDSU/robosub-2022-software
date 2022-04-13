"""
Server connects to Client and displays read frames in window.
Meant to be run as the main file, not being imported. 
"""
import socket
import cv2
import pickle
import struct

def main() -> None:
    """Runs server code for our client to have a connection, server receives and displays frames
    """
    HOST = ''
    PORT = 1234
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('Socket created')
    s.bind((HOST, PORT))
    print('Socket bind complete')
    s.listen(5)
    print('Socket now listening')
    conn, addr = s.accept()
    data = b""
    payload_size = struct.calcsize(">L")
    print("payload_size: {}".format(payload_size))
    
    # Display frames and print
    while True:
        while len(data) < payload_size:
            print("Recv: {}".format(len(data)))
            data += conn.recv(4096)

        print("Done Recv: {}".format(len(data)))
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        print("msg_size: {}".format(msg_size))
        while len(data) < msg_size:
            data += conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        # Frames displayed
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break


if __name__ == '__main__':
    main()
