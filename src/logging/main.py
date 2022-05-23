import logging
import os
import datetime
import platform
import sys


def existingLog():
    files = os.listdir()
    max = len(files)
    i = 0
    while i < max:
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

            break
        else:
            i += 1

class logger():
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


def main():
    logging.basicConfig(filename='last.log', level=logging.DEBUG,filemode='a',
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
    existingLog()
    main()
else:
    print('Erorr starting Log')
    sys.exit(1)
