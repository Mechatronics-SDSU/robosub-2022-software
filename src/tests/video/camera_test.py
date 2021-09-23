"""
camera_test.py tests camera functionality w/ varying resolutions
"""
import cv2
from datetime import datetime


def test_camera(cam: cv2.VideoCapture, input_res: str):
    """
    resizes frames according to input res. Supports screenshotting
    :param cam:         VideoCapture, device
    :param input_res:   str, {width}x{height}
    :return:            n/a
    """
    res = list(map(int, input_res.split('x')))  # width, height
    img_name = f'{res}_{datetime.now()}'

    while True:
        access, frame = cam.read()
        if not access:
            print("!Can't open camera")
            break
        frame = cv2.resize(frame, res, interpolation=cv2.INTER_AREA)
        cv2.imshow(img_name, frame)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key & 0xFF == ord('Q'):
            cam.release()
            break
        elif key & 0xFF == ord(' '):
            cv2.imwrite(f'{img_name}.png', frame)

    cv2.destroyWindow(img_name)
    cv2.destroyAllWindows()


if __name__ == '__main__':

    # test cases
    # for Linux, use cv2.CAP_V4L2 in VideoCapture() parameters
    test_camera(cv2.VideoCapture(0), '1080x720')
    test_camera(cv2.VideoCapture(0), '640x360')
    test_camera(cv2.VideoCapture(0), '1080x300')
    test_camera(cv2.VideoCapture(0), '700x1000')