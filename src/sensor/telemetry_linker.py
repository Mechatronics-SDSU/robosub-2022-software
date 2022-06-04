"""Handles gathering, organizing, and packaging of all telemetry data.
"""

import copy
import pickle
import socket
from multiprocessing import shared_memory as shm


TELEMETRY_DEFAULT_DATA = (0.0, 0.0, 0.0, 0.0)

telemetry_names_from_val = {
    2: 'pitch',
    4: 'roll',
    8: 'yaw',
    16: 'depth',
}

telemetry_vals_from_name = {
    'pitch': 2,
    'roll': 4,
    'yaw': 8,
    'depth': 16
}

telemetry_data = {
    1: 0,  # Integer used to show everything enabled
    2: TELEMETRY_DEFAULT_DATA[0],
    4: TELEMETRY_DEFAULT_DATA[1],
    8: TELEMETRY_DEFAULT_DATA[2],
    16: TELEMETRY_DEFAULT_DATA[3]
}


class TelemetryLinker:
    """Instantiates shared memory related to loading, saving, sending, receiving telemetry data.
    """
    def __init__(self):
        self.data = copy.deepcopy(telemetry_data)

    def load_shm(self, shm_val: int, value: any) -> None:
        """Load shared memory located at shm_val with MSB to LSB byte-ordered values in list.
        """
        pass

    def load_all(self) -> None:
        """Load shared memory, REGARDLESS OF IF DATA HAS BEEN PROPERLY UPDATED ELSEWHERE, into self.data.
        """

    def get_shm(self, shm_val: int) -> any:
        """Get what is stored at shm_val with MSB to LSB byte-ordered values in list.
        """
        pass

    def pack_shm(self) -> any:
        """Load all data the class has into a pickle, set integer after comparing to default values.
        """
        pass

    def unpack_shm(self) -> any:
        """Unpack a pickle with all data.
        """


def telemetry_server():
    """Start a socket server for GUI to access for telemetry data.
    """
    pass


if __name__ == '__main__':
    telemetry_server()
