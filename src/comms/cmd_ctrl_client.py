"""Command and control client.
Provides wrappers for the GUI to use to connect to the command and control server.
Meant to be imported from GUI, Can be run standalone to test.

Protocol:
Pickled array for configuration.
"""

import sys
import socket


def cmd_ctrl_client(host: str, port: int) -> None:
    """Driver code for the command and control server.
    """
    server_conn = False
    rcon_try_count = 0
    rcon_try_count_max = 3
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.connect((host, port))
            server_conn = True
            print('Connection to CNC server established')
        except ConnectionRefusedError:
            rcon_try_count += 1
            server_conn = False
            if rcon_try_count >= rcon_try_count_max:
                started = False
                print('Reconnect fail max exceeded, closing...')
                sys.exit(1)
        try:
            s.send(b'64')
            print('Sent message')
        except (ConnectionAbortedError, ConnectionResetError):
            print('Connection to server lost...')
            server_conn = False


if __name__ == '__main__':
    if len(sys.argv) > 2:
        cmd_ctrl_client(host=sys.argv[1], port=int(sys.argv[2]))
