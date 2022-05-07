"""Implements a client for receiving pilot control input on the SUB.
Pilot_Testing client accepts a socket connection and receives controller input from HOST.
This is encapsulated in a Controller object for convenient method access with
options for configurations.
Developed and tested with Ian's Xbox One Controller but other configuations can
be added at a later time.
Controller Note:
Because not all controllers are made with the same layouts for the right button pad,
they are named N S E W in the class, as to represent a compass.
This is to prevent confusion between different configurations and mappings.
"""

from __future__ import print_function
import os
import sys
import socket
import struct


import utils.maestro_driver as maestro_driver

server_port = 50004

def run_client() -> None:
    """Client's driver code
    """
    maestro = None  # Maestro object
    dev = "/dev/ttyACM0"
    if (os.name != 'nt') and (len(sys.argv) > 1):  # Windows check and see if we got a device
        dev = sys.argv[1].replace(' ', '')
    maestro = maestro_driver.MaestroDriver()
    started = True
    data = None
    payload_size = struct.calcsize('>8b')
    # Socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', server_port))
        print(f'Started controller server listening on port {server_port}, now listening... ')
        s.listen()
        conn, address = s.accept()
        
        while True:
            try:
                conn.sendall(b'1')  # Server ready to receive
            except BrokenPipeError:
                break  # HOST closed, restart this function to listen for new connection
            try:
                data = conn.recvfrom(1024)[0]
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
            except ConnectionAbortedError:
                break  # HOST closed, restart this function to listen for new connection
            if data is not None:
                data = struct.unpack('>8b', packed_msg_size)
                if dev is None:  # Print because we have no device
                    print(data)
                else:  # Have a device connected to this, send to Maestro
                    maestro.set_thrusts(data)


if __name__ == '__main__':
    run_client()
else:
    sys.exit()