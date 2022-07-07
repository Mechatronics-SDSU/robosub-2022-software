"""Contains main Dvl class to connect to Wayfinder.
"""
import datetime as dt
from sensor.dvl.packets import AppLayerPacket
from sensor.dvl.commands import BinaryCommands, check_response
from sensor.dvl.system import SystemInfo, SystemComponents, SystemFeatures, SystemSetup, \
    SystemTests, FftData, OutputData
from sensor.dvl.commands import ResponseStatusType, CommandIdType
import sys
import time
import numpy as np
import math
import struct

class Dvl():
    """Main class to connect to Wayfinder.
    """
    #pylint: disable=too-many-public-methods
    #pylint: disable=too-many-instance-attributes

    def __init__(self, com=None, baudrate=115200):
        self._commands = BinaryCommands()
        self._system_tests = SystemTests()
        self._system_setup = SystemSetup()
        self._system_info = SystemInfo()
        self._system_components = SystemComponents()
        self._system_features = SystemFeatures()
        self._fft_data = FftData(None)
        self._is_connected = False
        self.last_err = ResponseStatusType.SUCCESS
        """Last response from the system as dvl.commands.ResponseStatusType."""
        self.working_folder = None
        """Path to working folder where data will be stored."""
        self.log_file_name = None
        """Automatically generated file name when data logging starts."""
        self.log_all_data = False
        """Flag to turn on logging of all data for debugging purposes."""
        self.time_diff = 0
        """Time different between system time and PC time."""
        if com is not None:
            self.connect(com, baudrate)

    def __enter__(self):
        """Opens serial port on enter.
        """
        self._commands.port.__enter__()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Closes serial port on exit.
        """
        self._commands.port.__exit__(exc_type, exc_value, traceback)

    @property
    def system_info(self) -> SystemInfo:
        """After successful call to get_system results are stored in here."""
        return self._system_info

    @property
    def system_components(self) -> SystemComponents:
        """After successful call to get_components results are stored in here."""
        return self._system_components

    @property
    def system_setup(self) -> SystemSetup:
        """After successful call to get_setup results are stored in here."""
        return self._system_setup

    @property
    def system_tests(self) -> SystemTests:
        """After successful call to get_tests results are stored in here."""
        return self._system_tests

    @property
    def system_features(self) -> SystemFeatures:
        """After successful call to get_features results are stored in here."""
        return self._system_features

    @property
    def fft_data(self) -> FftData:
        """After successful call to get_fft_test results are stored in here."""
        return self._fft_data

    def connect(self, com: str, baud_rate: int = 115200) -> bool:
        """Connects to Wayfinder DVL.

        Parameters
        ----------
        com : str
            String that represents COM port to be opened, for example "COM1".
        baud_rate : int
            Baud rate to use when opening the port.

        Returns
        -------
        bool
            True if port is opened, False otherwise.
        """
        self._system_tests = SystemTests()
        self._system_setup = SystemSetup()
        self._system_info = SystemInfo()
        self._system_componets = SystemComponents()
        self._system_features = SystemFeatures()
        self._fft_data = FftData(None)
        self.last_err = ResponseStatusType.SUCCESS
        if self.log_all_data and self.working_folder is not None:
            name = "COM" + "_"
            self._commands.all_data_logger.open_file(self.working_folder, name, ".txt")
        self._commands.reset()
        port_opened = self._commands.port.open(com, baud_rate)
        self._is_connected = port_opened
        if port_opened:
            # Check if we can communicate with Wayfinder
            if not self.get_system():
                # Close port
                self.disconnect()
        else:
            self.last_err = ResponseStatusType.CANNOT_OPEN_PORT
        return self._is_connected

    def disconnect(self):
        """Disconnects from Wayfinder DVL.
        """
        self.stop_logging()
        self._commands.all_data_logger.close_file()
        self._commands.port.__exit__(None, None, None)
        self._is_connected = False

    def is_connected(self) -> bool:
        """Checks if system is connected.

        Returns
        -------
        bool
            True if system is connected, False otherwise.
        """
        return self._is_connected

    def get_time(self) -> dt:
        """Gets system time.

        Returns
        -------
        datetime
            Time of the system if successful, None otherwise.
        """
        (self.last_err, date_time) = self._commands.get_time()
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            if date_time is not None:
                self.time_diff = date_time - dt.datetime.now()
            return date_time
        return None

    def set_time(self, date_time: dt) ->bool:
        """Sets system time.

        Parameters
        ----------
        date_time : datetime
            Current time to be set.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.set_time(date_time)
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            self.get_time()
            return True
        return False

    def enter_command_mode(self) -> bool:
        """Enters command mode (stops pinging).

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.enter_command_mode()
        return self.last_err.value == ResponseStatusType.SUCCESS.value

    def exit_command_mode(self) -> bool:
        """Exits command mode (starts pinging).

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.exit_command_mode()
        return self.last_err.value == ResponseStatusType.SUCCESS.value

    def send_software_trigger(self) -> bool:
        """Sends software trigger if software trigger is on.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.send_software_trigger()
        return self.last_err.value == ResponseStatusType.SUCCESS.value


    def reset_to_defaults(self) -> bool:
        """Resets to factory defaults.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.reset_to_defaults()
        return self.last_err.value == ResponseStatusType.SUCCESS.value

    def set_speed_of_sound(self, value: float) -> bool:
        """Sets speed of sound value.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.set_speed_of_sound(value)
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            return True
        return False

    def get_tests(self) -> bool:
        """Performs and gets system tests results.  The results are in system_tests.
        It is required to send enter_command_mode() before using this command.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        (self.last_err, tests) = self._commands.get_tests()
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            self._system_tests = tests
            return True
        self._system_tests = SystemTests()
        return False

    def start_tests(self):
        """Starts system tests without waiting for results.
        """
        self._commands.send_cmd_without_wait(CommandIdType.GET_TESTS)

    def get_features(self) -> bool:
        """Gets system features.  The results are in system_features.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        (self.last_err, features) = self._commands.get_features()
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            self._system_features = features
            return True
        self._system_features = SystemFeatures()
        return False

    def set_system_features(self, feature_code: bytearray) ->bool:
        """Set system features.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.set_system_features(feature_code)
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            return True
        return False

    def get_setup(self) -> bool:
        """Gets user setup.  The results are in system_setup.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        (self.last_err, setup) = self._commands.get_setup()
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            self._system_setup = setup
            return True
        self._system_setup = SystemSetup()
        return False

    def set_setup(self, setup: SystemSetup) -> bool:
        """Sets user setup.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.set_setup(setup)
        return self.last_err.value == ResponseStatusType.SUCCESS.value

    def get_system(self) -> bool:
        """Gets system information.  The results are in system_info.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        (self.last_err, result) = self._commands.get_system()
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            self._system_info = result
            return True
        self._system_info = SystemInfo()
        return False

    def get_components(self) -> bool:
        """Gets hardware components information.  The results are in system_components.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        (self.last_err, result) = self._commands.get_components()
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            self._system_components = result
            return True
        self._system_components = SystemComponents()
        return False

    def get_fft_test(self) -> bool:
        """Gets FFT test.  The results are in fft_data.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        (self.last_err, result) = self._commands.get_fft_test()
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            self._fft_data = result
            if self._fft_data is not None and self._fft_data.is_valid:
                self._fft_data.process()
            return True
        self._fft_data = FftData(None)
        return False

    def _start_system_update(self, file_size: int, chunk_size: int) -> bool:
        """Starts firmware update.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.start_system_update(file_size, chunk_size)
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            return True
        return False

    def _upload_file(self, arr: bytearray, chunk_size: int) -> bool:
        """Uploads chunk of the file.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.last_err = self._commands.upload_file(arr, chunk_size)
        if self.last_err.value == ResponseStatusType.SUCCESS.value:
            return True
        return False

    def reset_fft(self):
        """Resets FFT queue.
        """
        self._commands.flush_fft_queue()

    def get_status_count(self) -> int:
        """Gets count of status packets.
        """
        return self._commands.get_status_count()

    def get_status_packet(self) -> AppLayerPacket:
        """Gets status from queue.  This function should be used after
        starting tests and waiting for status.

        Returns
        -------
        AppLayerPacket
            Status packet if successful, None otherwise.
        """
        if self.get_status_count() > 0:
            return self._commands.get_status_packet(0)
        return None

    def get_test_response(self) -> bool:
        """Gets command response for system test.
        Use this function after start_system_tests and checking test status.
        """
        response = self._commands.get_cmd_packet()
        err = check_response(response, CommandIdType.GET_TESTS)
        self.last_err = err
        if err.value == ResponseStatusType.SUCCESS.value:
            status = SystemTests()
            status.decode(response)
            self._system_tests = status
            return True
        return False

    def set_working_folder(self, working_folder: str):
        """Sets working folder for data logging.

        Parameters
        ----------
        working_folder : str
            A full path name of the folder where data will be logged.
            Note: Do not use shell extensions.
        """
        self.working_folder = working_folder

    def start_logging(self, working_folder: str, prefix: str) -> bool:
        """Starts logging data to working folder.

        Parameters
        ----------
        working_folder : str
            A full path name of the folder where data will be logged.
            Note: Do not use shell extensions.
        prefix : str
            Prefix to use for automatically generated file name.

        Returns
        -------
        bool
            True if successful, False otherwise.
        """
        self.log_file_name = self._commands.data_logger.open_file(working_folder, prefix)
        return self.log_file_name is not None

    def get_log_file_name(self):
        """Returns name of log file.

        Returns
        -------
        str
            Full file name of the logging file if on, None otherwise.
        """
        return self.log_file_name

    def stop_logging(self):
        """Stops data logging.
        """
        self._commands.data_logger.close_file()

    def is_logging(self):
        """Checks if data are logged.

        Returns
        -------
        bool
            True if data logging is on, False otherwise.
        """
        return self._commands.data_logger.is_logging()

    def register_ondata_callback(self, func, obj=None):
        """Registers on data received callback function.

        The callback function should be define as follows:
        def func(output_data: dvl.system.OutputData, obj):  where output_data are
        received by driver, and obj is any object.
        """
        self._commands.register_ondata_callback(func, obj)

    def unregister_all_callbacks(self):
        """Unregisters all callback functions.
        """
        self._commands.unregister_all_callbacks()

    def reset(self):
        """Resets queues and decoder.
        """
        self._commands.reset()

    def change_baud_rate(self, baud_index: int) -> bool:
        """Changes baud rate on open port.
        """
        baudrate = 115200 if baud_index == 7 else 9600
        return self._commands.port.set_baudrate(baudrate)



class Dvl_sample:
    """This class stores values from the last dvl sample processed by the
    dvl callback method.
    """
    def __init__(self):
        """Initializes variables for last dvl sample
        """
        self.time = 0.0
        self.data = []
    
    def set_data(self, _time:float, _data:list):
        self.time = _time
        self.data = _data

    def get_time(self) -> float:
        return self.time

    def get_data(self) -> list:
        return self.data


def dvl_data_callback(self, output_data: OutputData, obj: Dvl_sample):
    """Enters upon each software triggered ping to organize time, velocity,
    and range data.
    Returns:
        time: A string containing the time of last sample
        ret: A list of float32 containing the velocities in x,y,z axis and mean range to floor
    """
    if output_data is not None:
        time_raw = output_data.get_date_time()
        time = time_raw.strftime('%H:%M:%S.%f')[:-3]
        data = [output_data.vel_x, output_data.vel_y, output_data.vel_z,  output_data.mean_range]
        # ret = []
        # for i in data:
        #     ret.append(struct.pack(">1f", i))
        obj.set_data(time, data)

    print("No output data connection")
        
def print_data_callback(output_data: OutputData, obj):
    """Prints velocity, floor range, and time data to screen
    """
    del obj
    if output_data is not None:
        time = output_data.get_date_time()
        txt = time.strftime('%H:%M:%S.%f')[:-3]
        vels = np.array([output_data.vel_x, output_data.vel_y, output_data.vel_z])
        print("Got data {0}".format(txt))
        print(f"Velocities X: %9.3f Y: %9.3f Z: %9.3f" % (vels[0], vels[1], vels[2]))
        print(f"Mean Range: %9.3f" % output_data.mean_range)
        

def main(com_port:str) -> None:
    """Test Driver for basic DVL functionality
    """
    dvl = Dvl(com=com_port)
    dvl_data = Dvl_sample()
    dvl.reset_to_defaults()

    dvl.get_setup()
    dvl.system_setup.software_trigger = 1 # Enable for manual software triggered data ping rate

    dvl.enter_command_mode()
    twoseconds = dt.datetime.now() + dt.timedelta(seconds=2)
    timetarget = dt.datetime(twoseconds.year, twoseconds.month, twoseconds.day,
        twoseconds.hour, twoseconds.minute, twoseconds.second)
    now = dt.datetime.now()
    time.sleep((timetarget - now).total_seconds())
    
    dvl.set_time(dt.datetime.now())

    dvl.register_ondata_callback(print_data_callback, None)

    dvl.exit_command_mode()

    while True:
        if not dvl.send_software_trigger():
            print("Failed to send software trigger")
        else:
            print("Successfully sent software trigger")
        time.sleep(0.1)

if __name__ == '__main__':
    if len(sys.argv) > 1:
        dvl_dev_port = sys.argv[1].replace(' ', '')
        main(com_port=dvl_dev_port)
    else:
        print("Error: not enough arguments. (Was the dvl dev port passed?)")
        sys.exit(1)
