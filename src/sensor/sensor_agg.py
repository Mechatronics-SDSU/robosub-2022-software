"""Scion sensor aggregator.
"""
import time
from multiprocessing import shared_memory as shm
import ahrs as scion_ahrs
import depth_sensor as scion_ds


def aggregation() -> None:
    """Driver code for the sensor aggregator
    """
    # Depth sensor, holds 1 4-byte float
    try:
        depth_shm = shm.SharedMemory(create=True, size=4, name='depth_sensor_shm')
    except FileExistsError:
        depth_shm = shm.SharedMemory(name='depth_sensor_shm')
        depth_shm.unlink()
        depth_shm = shm.SharedMemory(create=True, size=4, name='depth_sensor_shm')
    # AHRS, holds 2 signed 1-byte integers (pitch, roll), 1 signed 4-byte float (yaw)
    try:
        ahrs_shm = shm.SharedMemory(create=True, size=6, name='AHRS_shm')
    except FileExistsError:
        ahrs_shm = shm.SharedMemory(name='depth_sensor_shm')
        ahrs_shm.unlink()
        ahrs_shm = shm.SharedMemory(create=True, size=6, name='depth_sensor_shm')
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
    pass


if __name__ == '__main__':
    sensor_server()
