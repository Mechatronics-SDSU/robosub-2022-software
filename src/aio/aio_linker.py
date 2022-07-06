#!/usr/bin/env python3.8
"""AIO linker for GUI and listener.
"""
import copy
from multiprocessing import shared_memory as shm

aio_data = [
    0,
    0,
    0,
    0,
    0,
    0
]


class AIOLinker:
    def __init__(self, use_shm=True):
        self.data = copy.deepcopy(aio_data)
        if use_shm:
            self.shm_objects = []
            self.setup()

    def setup(self):
        """Set up shared memory
        """
        pass


