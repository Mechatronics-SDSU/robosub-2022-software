"""Example usage for DVL driver
"""
from sensor.dvl.dvl import Dvl
from sensor.dvl.system import OutputData
import sys
import numpy as np

def update_data(output_data: OutputData, obj):
    """Prints data time to screen
    """
    del obj
    if output_data is not None:
        time = output_data.get_date_time()
        txt = time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        vels = np.array([output_data.vel_x, output_data.vel_y, output_data.vel_z])
        print("Got data {0}".format(txt))
        print(f"Velocities X: %9.3f Y: %9.3f Z: %9.3f" % (vels[0], vels[1], vels[2]))
        print(f"Mean Range: %9.3f" % output_data.mean_range)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        dvl_dev_port = sys.argv[1].replace(' ', '')
    else:
        print("Error: not enough arguments. (Was the dvl dev port passed?)")
        sys.exit(1)

    # Connect to serial port
    with Dvl(dvl_dev_port, 115200) as DVL:

        if DVL.is_connected():

            # Get user system setup
            if DVL.get_setup():
                # Print setup 
                print (DVL.system_setup)

            # Stop pinging
            if not DVL.enter_command_mode():
                print("Failed to stop pinging")

            # Reset to factory defaults (requires Wayfinder to be in 'command mode')
            if not DVL.reset_to_defaults():
                print("Failed to reset to factory defaults")

            # Register callback function
            DVL.register_ondata_callback(update_data)

            # Start pinging
            if not DVL.exit_command_mode():
                print("Failed to start pinging")

            # Blocking call to wait for key pressed to end program
            KEY = input("Press Enter to stop\n")

        else:
            print("Failed to open {0} - make sure it is not used by any other program".format(PORT))

        # Unregister
        DVL.unregister_all_callbacks()