"""Quick script to get the unique serial numbers of
all serial devices connected over USB. Should be run
as main. Usefull for killswitch.py, where the S/N is
needed to communicate with the killswitch.
"""

import serial.tools.list_ports

def main() -> None:
    """
    Prints out all serial USB devices, their locations, and their serial numbers.
    """

    print("""
              ¤═══════════════════════════════¤
              ║       Available Devices       ║
              ¤═══════════════════════════════¤
    """)

    for port in serial.tools.list_ports.comports():
        print("Device name: " + port.description + 
        "S/N: " + port.serial_number + 
        "  Port: " + port.device)

    print("""\n
              ¤═══════════════════════════════¤\n""")


if __name__ == "__main__":
    main()
