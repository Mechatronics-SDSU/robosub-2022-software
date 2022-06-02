"""
A better driver program for the kill switch.
Handles automatic USB discovery (no more ID required),
automatically forks the server to a new process,
provides simple arguements to control switch: on, off, get,
and stop.

Usage:
    python3 killswitch-aio.py [on,off,get,stop]

    * on    Activates the switch, same as pressing the kill button
    * off   Sets the killswitch to false
    * get   Gets the current status of the switch (True, False)
    * stop  Stops the killswitch server and closes the socket

This shall be run as main and not as part of another program!
"""

import serial.tools.list_ports
import time
import socket
import sys
import os

SOCK_PATH = "/tmp/killswitch_sock"


def client() -> None:
    """Client method, talks to the kill switch server
    and parses arguements
    """
    op = 2
    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg == "on":
            op = 0
        elif arg == "off":
            op = 1
        elif arg == "get":
            op = 2
        elif arg == "stop":
            op = 3

    client_sock = socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)

    try:
        time.sleep(1)
        client_sock.connect(SOCK_PATH)
        client_sock.send(str(op).encode())
        print(client_sock.recv(128).decode())
    # except Exception:
        # print("something went wrong, aborting!")
    except KeyboardInterrupt:
        print("Keyboard Interrupt, aborting!")
    finally:
        client_sock.close()
        exit()


def server() -> None:
    """Server method, handles serial communication
    (keeps connection alive, uses Killswitch class, etc.)
    sends info about killswitch to client
    """
    if os.path.exists(SOCK_PATH):
        os.remove(SOCK_PATH)
    serv_sock = socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    killswitch = Killswitch()
    serv_sock.bind(SOCK_PATH)
    serv_sock.listen()
    try:
        while 1:
            conn,addr = serv_sock.accept()
            data = conn.recv(128)
            op = int(data)
            if op == 0:
                killswitch.set_status(True)
                conn.send(b"Activating kill switch. State: " 
                + bytes(str(killswitch.get_status()).encode("UTF-8")))
            elif op == 1:
                killswitch.set_status(False)
                conn.send(b"Deactivating kill switch. State: " 
                + bytes(str(killswitch.get_status()).encode("UTF-8")))
            elif op == 2:
                conn.send(b"Current Killswitch State: " 
                + bytes(str(killswitch.get_status()).encode("UTF-8")))
            elif op == 3:
                conn.send(b"Stopping kill switch server")
                conn.close()
                break
    except KeyboardInterrupt:
        print("Exiting!")
    finally:
        serv_sock.close()  
        os.unlink(SOCK_PATH)  


class Killswitch:
    """Class to interact with the prototype kill switch for Scion.
    Uses Serial to send 3 bytes (operation identifier, value, and newline)
    to the board, which acts accordingly. Allows the kill switch to be 
    accessed and modified through python.
    """
    def __init__(self) -> None:
    
        self.killswitch_device = ""
        for port in serial.tools.list_ports.comports():
        
            with serial.Serial(port.device,9600) as device:
                if "killswitch" in str(device.readline()):
                    self.killswitch_device = port.device
        try:
            print("=>Attempting to connect to killswitch at..." 
            + self.killswitch_device)
            self.killswitch = serial.Serial(self.killswitch_device, 9600)
            time.sleep(.75)
        except AttributeError:
            print("=>Failed to locate kill switch. Did the device change?")
            exit()
        except serial.serialutil.SerialException:
            print("=>Could not connect to identified device! Aborting.")
            exit()
      
    def get_status(self) -> bool:
        """Uses serial over USB to get the current status of the kill switch.
        """
        try:
            while True:
                self.killswitch.write(b'g0\n')
                response = self.killswitch.readline()
                # print(f'RESPONSE RAW: {response}')
                if response[0] == ord('r'):
                    return bool(response[1] - 48)
                    break
        except AttributeError:
            print("=>Failed to get the kill switch status, \
is the device disconnected?")
        except serial.SerialException:
            print("=>Did not recieve any data from the serial \
device, is the device disconnected?")

    def set_status(self,new_val: bool) -> None:
        """Uses serial over USB to change the state of the kill switch.
        """
        try:
            while True:
                self.killswitch.write(b's' + bytes([new_val]) + b'\n')
                response = self.killswitch.readline()
                if response[0] == ord('r') and \
                bool(response[1] - 48) == new_val:
                    break
        except AttributeError:
            print("=>Failed to set the kill switch status, \
is the device disconnected?")
        except serial.SerialException:
            print("=>Did not recieve any data from the serial \
device, is the device disconnected?")

    def end(self) -> None:
        """Closes the serial connection when done"""
        try:
            self.killswitch.close()
        except AttributeError:
            print("=>Failed to access kill switch, is the device connected?")


if __name__ == "__main__":

    #Check if a UNIX socket for the server exists, if so then
    #we proceed to the client method. Otherwise we fork and
    #start the server in its own process.
    if os.path.exists("/tmp/killswitch_sock"):
        print("Socket already exists, connecting")
        client()
    else:
        pid = os.fork()
    if pid > 0:
        time.sleep(1)
        client()
    else:
        print("Server not found, starting...")
        server()
