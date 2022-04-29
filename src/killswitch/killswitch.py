"""Prototype Python driver for prototype kill switch on Scion.
This program implements basic 2-way communication over serial USB to the
kill switch. It provides the Killswitch class, which has methods for
retrieving and modifying a boolean value stored on the Arduino. It
also requires the hardware-specific serial number of whatever device
it is communicating with in order to find the right com port. Right now,
it has a simple text interface running in main().

¤══════════════════════════════════════════════════════¤
Basic use:

    import serial.tools.list_ports

    
    killswitch_id = "123456789"
    killswitch = Killswitch(killswitch_id)

    status = killswitch.get_status()

    if status == False:
        killswitch.set_status(True)
    elif status == True:
        killswitch.set_status(False)

    killswitch.close()

¤══════════════════════════════════════════════════════¤
How the Serial Communication works:

 [Computer] ────┐
                │
                │
                │ > g0\n (Code to retrieve current state from arduino. 'g' specifies Get)
                │ < r1\n (Response from Arduino. 'r' specifies response, '1' is boolean value)
                │ > s0\n (Code to modify the state on Arduino. 's' specifies Set, '0' is boolean )
                │ < r1\n (Response from Arduino with new state. 'r' specifies repsonse, '1' is boolean)
                │
                │
                └─── [Arduino]
¤══════════════════════════════════════════════════════¤
IMPORTANT NOTE
In order for this program to function, you need to include your device's serial number!
"""

import serial.tools.list_ports
import time

unique_ID = "95237323934351401192"


def main() -> None:
    """Runs some basic demo code for the Killswitch class
    """
    killswitch = Killswitch(unique_ID)
    print("""
    ¤═══════════════════════════════¤
        Kill Switch Driver Demo
         Type \'on\' \'off\' \'exit\'
    ¤═══════════════════════════════¤
        """)
    try:
        while True:
            print("Current state: " + str(killswitch.get_status()))
            user_sel = input(">> ");
            if user_sel == "on":
                killswitch.set_status(True)
            elif user_sel == "off":
                killswitch.set_status(False)
            elif user_sel == "exit":
                killswitch.end()
                print("Goodbye!")
                return
    except KeyboardInterrupt:
        killswitch.end()
        print("Goodbye!")
    
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
            print("Failed to get the kill switch status, is the device disconnected?")
        except serial.SerialException:
            print("Did not recieve any data from the serial device, is the device disconnected?")

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
