"""Generates anything in opencv with default text ahead of time.
"""
import cv2

import utils.scion_utils as scion_ut

color_term_green = (74, 246, 38)
color_error_red = (255, 0, 3)
sensor_comms_names = [
    'COMMS',
    'Command:',
    'Camera 0:',
    'Camera 1:',
    'Sensor:',
    'Pilot:',
    'Logging',
    'SENSORS',
    'Pitch',
    'Roll',
    'Yaw',
    'Depth'
]
uscore = '_'*24


def generate_sensor_comms_window():
    sensor_frame = cv2.imread('img/sensor_base_22.png')
    for i in range(len(sensor_comms_names)):
        cv2.putText(img=sensor_frame, text=sensor_comms_names[i], org=(8, scion_ut.RIGHT_WIN_CONF[i]),
                    fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.2, color=color_term_green, thickness=1)
    cv2.putText(img=sensor_frame, text=uscore, org=(8, 32),
                fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.2, color=color_term_green, thickness=1)
    cv2.putText(img=sensor_frame, text=uscore, org=(8, 207),
                fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.2, color=color_term_green, thickness=1)
    cv2.imwrite('img/sensor_22.png', sensor_frame)


if __name__ == '__main__':
    generate_sensor_comms_window()
