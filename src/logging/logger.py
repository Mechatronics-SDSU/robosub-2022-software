"""Logging process
Logs data to file with timestamp and renames already existing file.
Uses input() to get data directly from the terminal but will be ported to listen on ROS topics.
"""

import logging
import os
import datetime
import platform
import sys


def existing_log() -> None:
    """Existing log checks for an already existing log and renames it to last modification date.
    """
    files = os.listdir()
    max_len = len(files)
    i = 0
    while i < max_len:
        if files[i].startswith('last.log'):
            existinglogpath = os.path.join(os.getcwd(), files[i])
            if platform.system() == 'Windows':
                creation = datetime.datetime.fromtimestamp((os.path.getmtime(existinglogpath))).strftime('%Y-%m-%d-%H-%M-%S')
                os.rename('last.log', creation)
                break
            else:
                stat = os.stat(existinglogpath)
                try:
                    create = stat.st_birthtime
                    os.rename('last.log', create)
                    break
                except AttributeError:
                    create = datetime.datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d-%H-%M-%S')
                    os.rename('last.log', create)
                    break
        else:
            i += 1


class logger:
    """Logging wrapper for specifying what level the log is at
    """
    def __init__(self, data):
        self.data = str(data)

    def log(self):
        return self.data

    def debug(self, data):
        logging.debug(self.log())

    def info(self, data):
        logging.info(self.log())

    def error(self, data):
        logging.error(self.log())

    def warning(self, data):
        logging.warning(self.log())

    def critical(self, data):
        logging.critical(self.log())


def main() -> None:
    """Driver code for the logger class
    """
    logging.basicConfig(filename='last.log', level=logging.DEBUG, filemode='a',
                        format="%(levelname)s:[%(asctime)s]:%(message)s", datefmt='%H:%M:%S')
    data_logger = None
    while True:
        data = str(input())  # Change me for ROS
        if data_logger is None:
            data_logger = logger(data)
        else:
            data_logger.data = data
        if data_logger.data.startswith('DEBUG'):
            data_logger.data = data_logger.data[6:]
            data_logger.debug(data)
        elif data_logger.data.startswith('INFO'):
            data_logger.data = data_logger.data[5:]
            data_logger.info(data)
        elif data_logger.data.startswith('ERROR'):
            data_logger.data = data_logger.data[6:]
            data_logger.error(data)
        elif data_logger.data.startswith('WARNING'):
            data_logger.data = data_logger.data[8:]
            data_logger.warning(data)
        elif data_logger.data.startswith('CRITICAL'):
            data_logger.data = data_logger.data[9:]
            data_logger.critical(data)
        elif data_logger.data.startswith('quit'):
            break


if __name__ == '__main__':
    print('Starting Log')
    existing_log()
    main()
else:
    print('Erorr starting Log, don\'t import the logger!')
    sys.exit(1)
