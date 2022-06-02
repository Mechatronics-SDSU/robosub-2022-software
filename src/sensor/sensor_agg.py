"""Scion sensor aggregator.
"""
import socket
import time
import random
from multiprocessing import shared_memory as shm
import ahrs as scion_ahrs
import depth_sensor as scion_ds


def aggregation() -> None:
    """Driver code for the sensor aggregator
    """
    # Depth sensor, holds 1 4-byte float
    try:
        depth_shm = shm.SharedMemory(create=True, size=4, name='depth_sensor_shm_s')
    except FileExistsError:
        depth_shm = shm.SharedMemory(name='depth_sensor_shm_s')
        depth_shm.unlink()
        depth_shm = shm.SharedMemory(create=True, size=4, name='depth_sensor_shm_s')
    # AHRS, holds 2 signed 1-byte integers (pitch, roll), 1 signed 4-byte float (yaw)
    try:
        ahrs_shm = shm.SharedMemory(create=True, size=6, name='ahrs_shm_s')
    except FileExistsError:
        ahrs_shm = shm.SharedMemory(name='ahrs_shm_s')
        ahrs_shm.unlink()
        ahrs_shm = shm.SharedMemory(create=True, size=6, name='ahrs_shm_s')
    # Get wrappers
    ahrs = scion_ahrs.SpartonAHRSDataPackets("DVL NAME GOES HERE")
    depth = scion_ds.Depth()
    # Write to SHM
    while True:
        _pr = ahrs.get_pitch_roll()
        _y = bytes(ahrs.get_true_heading())
        ahrs_shm.buf[0] = _pr[0]  # Pitch
        ahrs_shm.buf[1] = _pr[1]  # Roll
        ahrs_shm.buf[2] = _y[0]  # Yaw
        ahrs_shm.buf[3] = _y[1]
        ahrs_shm.buf[4] = _y[2]
        ahrs_shm.buf[5] = _y[3]
        _d = depth.get_state()
        depth_shm.buf[0] = _d[0]
        depth_shm.buf[1] = _d[1]
        depth_shm.buf[2] = _d[2]
        depth_shm.buf[3] = _d[3]
        time.sleep(0.05)


def sensor_server() -> None:
    """Driver code for sending sensor data to GUI
    """
    depth_shm = shm.SharedMemory(name='depth_sensor_shm_s')
    ahrs_shm = shm.SharedMemory(name='ahrs_shm_s')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', 50003))
        s.listen()
        conn, address = s.accept()
        print('Sensor Aggregation Socket Listening...')
        while True:
            result = conn.recvfrom(1024)[0]
            if result == b'1':
                data = bytes([depth_shm.buf[0], depth_shm.buf[1], depth_shm.buf[2], depth_shm.buf[3], ahrs_shm.buf[0],
                              ahrs_shm.buf[1], ahrs_shm.buf[2], ahrs_shm.buf[3], ahrs_shm.buf[4], ahrs_shm.buf[5]])
                '''data = bytes([
                    random.randint(0, 127),
                    random.randint(0, 127),
                    random.randint(0, 127),
                    random.randint(0, 127),
                    random.randint(0, 127),
                    random.randint(0, 127),
                    random.randint(0, 127),
                    random.randint(0, 127),
                    random.randint(0, 127),
                    random.randint(0, 127)
                ])'''
                conn.sendall(data)


if __name__ == '__main__':
    sensor_server()
