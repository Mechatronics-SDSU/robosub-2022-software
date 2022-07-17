#!/usr/bin/env python3.8
"""AIO linker for GUI and listener.
"""
import copy
import pickle

AIO_DEFAULT_DATA = (1,  # Kill switch
                    0,  # Auto switch
                    0,  # Battery State
                    0,  # Leak Detection
                    0,  # Weapons
                    0  # Arm
                    )
AIO_DATA_SIZES = (1,  # Kill switch
                  1,  # Auto switch
                  1,  # Battery State
                  1,  # Leak Detection
                  1,  # Weapons
                  1  # Arm
                  )
aio_names_from_val = {
    0: 'kill',
    1: 'auto',
    2: 'battery',
    3: 'leak',
    4: 'weapons',
    5: 'arm'
}

aio_vals_from_name = {
    'kill': 0,
    'auto': 1,
    'battery': 2,
    'leak': 3,
    'weapons': 4,
    'arm': 5
}

aio_vals_from_char = {
    'k': 0,
    'a': 1,
    'b': 2,
    'l': 3,
    'w': 4,
    'r': 5
}

aio_chars_from_val = {
    0: 'k',
    1: 'a',
    2: 'b',
    3: 'l',
    4: 'w',
    5: 'r'
}

aio_data = []
for x in range(len(AIO_DEFAULT_DATA)):
    aio_data.append(AIO_DEFAULT_DATA[x])


class AIOLinker:
    def __init__(self):
        self.data = copy.deepcopy(aio_data)

    def set_current_state(self, system: str, state: int) -> None:
        sys_index = aio_vals_from_char[system]
        self.data[sys_index] = state

    def serialize(self) -> bytes:
        return pickle.dumps(self.data)

    def from_serial(self, input_data: bytes) -> None:
        if input_data != b'':
            self.data = pickle.loads(input_data)

    def from_serial_with_diff(self, input_data: bytes) -> list:
        """Serializes and returns 2 lists of what was changed.
        """
        if input_data != b'':
            in_data = pickle.loads(input_data)
        else:
            return [[], []]
        out = [[], []]  # Output is a tuple of ros characters and entries
        for i in range(len(self.data)):
            if in_data[i] != self.data[i]:
                out[0].append(aio_chars_from_val[i])  # Character for ROS topic
                out[1].append(in_data[i])  # Data entry
        self.data = in_data
        return out
