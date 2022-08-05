"""Command and control client.
Provides wrappers for the GUI to use to connect to the command and control server.
Meant to be imported from GUI, Can be run standalone to test.

Protocol:
Pickled array for configuration.
"""

import sys
import socket


class CNCWrapper:
    def __init__(self, host: str, port: int, debug: False) -> None:
        self.host = host
        self.port = port
        self.debug = debug
        self.server_conn = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.setup_connection()

    def setup_connection(self) -> None:
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.connect((self.host, self.port))
            self.server_conn = True
            if self.debug:
                print('Connection to CNC Server Established.')
        except ConnectionRefusedError:
            self.server_conn = False

    def send_message(self, message: bytes) -> None:
        try:
            self.sock.send(message)
        except (ConnectionAbortedError, ConnectionResetError):
            print('Connection to server lost...')
            self.server_conn = False


def cmd_ctrl_client(host: str, port: int, debug=False) -> None:
    """Driver code for the command and control server.
    """
    cnc = CNCWrapper(host=host, port=port, debug=debug)
    cnc.send_message(b'32')


if __name__ == '__main__':
    if len(sys.argv) > 2:
        cmd_ctrl_client(host=sys.argv[1], port=int(sys.argv[2]), debug=True)
