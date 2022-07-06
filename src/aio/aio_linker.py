#!/usr/bin/env python3.8
"""AIO linker for GUI and listener.
"""
import copy
from multiprocessing import shared_memory as shm

AIO_DEFAULT_DATA = (0,  # Kill switch
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

aio_data = []
for x in range(len(AIO_DEFAULT_DATA)):
    aio_data.append(AIO_DEFAULT_DATA[x])


class AIOLinker:
    def __init__(self, use_shm=True):
        self.data = copy.deepcopy(aio_data)
        if use_shm:
            self.shm_objects = []
            self.setup_shm()

    def setup_shm(self):
        """Set up all shared memory
        """
        for i in range(len(self.data)):
            try:
                shm_i = shm.SharedMemory(name=f'aio_{aio_names_from_val[i]}_shm')
            except FileNotFoundError:
                shm_i = shm.SharedMemory(create=True, size=AIO_DATA_SIZES[i],
                                         name=f'aio_{aio_names_from_val[i]}_shm')
            for j in range(AIO_DATA_SIZES[i]):  # Clear any existing data
                shm_i.buf[j] = 0
            self.shm_objects.append(shm_i)



