"""Handles gathering, organizing, and packaging of all telemetry data.
"""

import copy
import struct
import pickle
import socket
from multiprocessing import shared_memory as shm


TELEMETRY_DEFAULT_DATA = (0.0, 0.0, 0.0, 0.0)
TELEMETRY_DATA_SIZES = (8, 8, 8, 8)

telemetry_names_from_val = {
    0: 'pitch',
    1: 'roll',
    2: 'yaw',
    3: 'depth',
}

telemetry_vals_from_name = {
    'pitch': 0,
    'roll': 1,
    'yaw': 2,
    'depth': 3
}

telemetry_data = [
    TELEMETRY_DEFAULT_DATA[0],
    TELEMETRY_DEFAULT_DATA[1],
    TELEMETRY_DEFAULT_DATA[2],
    TELEMETRY_DEFAULT_DATA[3]
]


class TelemetryLinker:
    """Instantiates shared memory related to loading, saving, sending, receiving telemetry data.
    """
    def __init__(self):
        self.data = copy.deepcopy(telemetry_data)
        self.shm_objects = []
        self.setup()

    def setup(self):
        """Set up all shared memory
        """
        for i in range(len(self.data)):
            try:
                shm_i = shm.SharedMemory(name=f'{telemetry_names_from_val[i]}_sensor_shm')
            except FileNotFoundError:
                shm_i = shm.SharedMemory(create=True, size=TELEMETRY_DATA_SIZES[i],
                                         name=f'{telemetry_names_from_val[i]}_sensor_shm')
            for j in range(TELEMETRY_DATA_SIZES[i]):  # Clear any existing data
                shm_i.buf[j] = 0
            self.shm_objects.append(shm_i)

    def _set_shm(self, shm_val: int, value: any) -> None:
        """Load shared memory located at shm_val with MSB to LSB byte-ordered values in list.
        """
        ba = bytearray(struct.pack('d', value))
        for i in range(len(ba)):
            self.shm_objects[shm_val].buf[i] = ba[i]

    def load_data(self, shm_index: int, value: any) -> None:
        """Load data by string into the class, then call internal _set_shm to set it in shm.
        """
        self.data[shm_index] = value
        self._set_shm(shm_index, value)

    def load_data_str(self, shm_str: str, value: any) -> None:
        """Load data by string into the class, then call internal _set_shm to set it in shm.
        """
        index = telemetry_vals_from_name[shm_str]
        self.data[index] = value
        self._set_shm(index, value)

    def load_all(self) -> None:
        """Load shared memory, REGARDLESS OF IF DATA HAS BEEN PROPERLY UPDATED ELSEWHERE, into self.data.
        """
        for i in range(len(self.data)):
            self.data[i] = self.get_shm(i)

    def get_shm(self, shm_val: int) -> any:
        """Get what is stored at shm_val with MSB to LSB byte-ordered values in list.
        """
        ba = bytearray()
        for i in range(TELEMETRY_DATA_SIZES[shm_val]):
            ba.append(self.shm_objects[shm_val].buf[i])
        return struct.unpack('d', ba)[0]

    def pack_shm(self) -> any:
        """Load all data the class has into a pickle, set integer after comparing to default values.
        """
        return pickle.dumps(self.data)

    def unpack_shm(self, loading_pickle: bytes) -> any:
        """Unpack a pickle with all data, load into shm
        """
        data = pickle.loads(loading_pickle)
        print(data)
        for i in range(len(data)):
            self.load_data(i, data[i])


def telemetry_server():
    """Start a socket server for GUI to access for telemetry data.
    """
    tel = TelemetryLinker()


if __name__ == '__main__':
    telemetry_server()
