"""Implements a server for receiving pilot control input on Scion.
This server accepts a socket connection and receives controller input from HOST.
Developed and tested with Ian's Xbox One Controller but other configuations can be added at a later time.
Testing note 5/7/22: This was successfully run on the team's xbox controllers for the 5/7/22 pool test.
"""

from __future__ import print_function
import os
import sys
import socket
import struct


import utils.maestro_driver as maestro_driver

server_port = 50004


def run_server() -> None:
    """Server's driver code, runs entirely within this function
    """
    print('Starting Controller Server...')
    maestro = None  # Maestro object
    dev = None
    if len(sys.argv) > 1:  # Did we get a device on program start?
        dev = sys.argv[1].replace(' ', '')
    else:
        print(f'Error: Expected argc > 1, number of args = {len(sys.argv)}. (Did you add the maestro device\'s COM '
              f'port?)')
        print('Exiting Controller Server...')
        sys.exit(1)
    maestro = maestro_driver.MaestroDriver(com_port=dev)
    print('Maestro driver initialized.')
    started = True
    data = None
    payload_size = struct.calcsize('>8b')
    # Socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', server_port))  # Bind to self, we're the ones hosting the server
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
    run_server()
else:
    print('Run server code as main!')
    sys.exit(1)
