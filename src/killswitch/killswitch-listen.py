"""Listener program for the killswitch, work in progress, UNTESTED.
Meant to be run from main() NOT IMPORTED
Error messages scilenced for readability reasons.
"""

import serial.tools.list_ports
import sys
import time

def main() -> None:
    if len(sys.argv) > 1:
        if sys.argv[1].lower() == "--id":
            if len(sys.argv) < 3 or not sys.argv[2].isdecimal():
                print("Invalid S/N.")
                sys.exit()
            
            killswitch = Killswitch(int(sys.argv[2]))
            ks_status = killswitch.get_status()
            print(ks_status)
            try:
                while True:
                    new_ks_status = killswitch.get_status()
                    if ks_status != new_ks_status:
                        ks_status = new_ks_status
                        print(ks_status)
                        time.sleep(0.05)
            except KeyboardInterrupt:
                sys.exit();
    else:
        print("Missing S/N, include with --id [S/N].")
        sys.exit()

class Killswitch:
    """Class to interact with the prototype kill switch for Scion.
    Uses Serial to send 3 bytes (operation identifier, value, and newline)
    to the board, which acts accordingly. Allows the kill switch to be 
    accessed and modified through python.
    """
    
    #Iterates through all com ports, checks for a unique ID.
    #If that unique ID is found, it stores the path to that device
    #in a variable for later use. If it successfully connects to the
    #device, it will wait 2 seconds before continuing (I ran into issues
    #immediately attempting to communicate with the device after creating
    #the serial connection, thus the 2 second delay).
    def __init__(self, killswitch_id: str) -> None:
        self.killswitch_id = killswitch_id
        for port in serial.tools.list_ports.comports():
            if port.serial_number == self.killswitch_id:
                self.usb_port = port.device       
        try:
            print("Attempting to connect to killswitch at" + self.usb_port)
            self.killswitch = serial.Serial(self.usb_port, 9600)
            time.sleep(2)
        except AttributeError:
            print("Failed to locate kill switch. Did the device change?")

    #Although the first byte (b'g') is the one that signals the arduino
    #to send the kill switch state, the 3 bytes sent(action,value,newline)
    #leaves room for sending other commands and values to the arduino and
    #simplfies the process of interpreting commands on the arduino itself.
    #The complete message sent over serial looks like 'g0\n'. The arduino
    #responds in a similar format of 'rX\n' where X is the boolean value.        
    def get_status(self) -> bool:
        """Uses serial over USB to get the current status of the kill switch.
        """
        try:
            while True:
                self.killswitch.write(b'g0\n')
                response = self.killswitch.readline()
                if response[0] == ord('r'):
                    return bool(response[1] - 48)
                    break
        except AttributeError:
            # print("Failed to get the kill switch status, is the device disconnected?")
            "silenced warning for readability reasons"
            
        except serial.SerialException:
            # print("Did not recieve any data from the serial device, is the device disconnected?")
            "scilenced for readability reasons"
            return None

    #As mentioned earlier, 3 bytes are sent to the arduino. In this case
    #'sX\n' is the command to modify the current state of the kill switch.
    #The first byte signals the action (s for SET), the second is the
    #boolean value, and the third is simply the terminator. This code
    #will attempt to set the kill switch value, read and verify the value,
    #and repeat if the value was not set.
    def set_status(self,new_val: bool) -> None:
        """Uses serial over USB to change the state of the kill switch.
        """
        try:
            while True:
                self.killswitch.write(b's' + bytes([new_val]) + b'\n')
                response = self.killswitch.readline()
                if response[0] == ord('r') and bool(response[1] - 48) == new_val:
                    break
        except AttributeError:
            print("Failed to set the kill switch status, is the device disconnected?")
        except serial.SerialException:
            print("Did not recieve any data from the serial device, is the device disconnected?")

    def end(self) -> None:
        """Closes the serial connection when done"""
        try:
            self.killswitch.close()
        except AttributeError:
            print("Failed to access kill switch, is the device connected?")




if __name__ == "__main__":
    main()
