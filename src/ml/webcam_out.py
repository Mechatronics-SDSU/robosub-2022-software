import numpy as np
import cv2
import PIL.Image
import sys
import pickle
import struct
import socket

from pydarknet import Detector, Image


def detector(cfg_file: str, weights_file: str, data_file: str) -> None:
    # OpenCV
    cap = cv2.VideoCapture(2)
    cap.set(3, 640)
    cap.set(4, 480)
    img_counter = 0
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    # Darknet
    net = Detector(bytes(cfg_file, encoding="utf-8"), bytes(weights_file, encoding="utf-8"), 0, bytes(data_file,
                                                                                                      encoding="utf-8"))
    # Socket
    print('Cameras ready for connection...')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('', 50001))
        server_sock.listen(5)
        print('SOCKET ONLINE FOR CAMERA...')
        conn, addr = server_sock.accept()
        while True:
            ret, img = cap.read()
            if ret is True:
                img = cv2.flip(img, 1)
                img2 = cv2.flip(img, 0)
                img3 = img2.astype(np.float32)
                img3 = Image(img3)
                print('Detecting...')
                results = net.detect(img3)
                print(results)
                results_label = [x[0] for x in results]
                if len(results_label) > 0:
                    c_y = results[0][2][0]
                    c_x = results[0][2][1]
                    h = results[0][2][2]
                    w = results[0][2][3]
                    print(c_x)
                    print(c_y)
                    print(h)
                    print(w)
                    x_1 = int(c_x - w / 2)
                    y_1 = int(c_y - h / 2)
                    x_2 = int(c_x + w / 2)
                    y_2 = int(c_y + h / 2)
                    img2 = cv2.rectangle(img2, (x_1, y_1), (x_2, y_2), (0, 0, 255), 2)
                    #img2 = cv2.rectangle(img=img2, pt1=(x_1, y_1), pt2=(x_2, y_2), color=(0, 0, 255), thickness=2)
                result, frame = cv2.imencode('.jpg', img2, encode_param)
                data = pickle.dumps(frame, 0)
                size = len(data)
                try:
                    conn.sendall(struct.pack(">L", size) + data)
                except:
                    print('Couldn\'t send frame')
                img_counter += 1


if __name__ == '__main__':
    if len(sys.argv) > 3:
        detector(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print('Error, argc not > 3. (Did you add scion.cfg, scion.weights, and scion.data?)')
        sys.exit(1)

