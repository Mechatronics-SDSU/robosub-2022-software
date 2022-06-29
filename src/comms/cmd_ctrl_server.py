"""Command and control server.
Sets configuration of Scion remotely via gui connection, read into masterprocess via input pipe redirection.
"""

import sys
import socket


CMD_CTRL_SERVER_PORT = 50000


def cmd_ctrl_server():
    """Driver code for the command and control server.
    """
    data = None
    # Start up socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', CMD_CTRL_SERVER_PORT))
        s.listen()
        conn, addr = s.accept()
        try:
            data = conn.recvfrom(1024)[0]
        except ConnectionAbortedError:
            print('Host closed from connection abort')
        if data is not None:
            data = int.from_bytes(data, 'big')
            sys.stdout.write(str(data) + '\0')


if __name__ == '__main__':
    cmd_ctrl_server()
