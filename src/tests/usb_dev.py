"""Tests all USB devices.
"""
import usb


def get_serial_dev(devid: int) -> None:
    """Print serial number from device ID.
    """
    dev = usb.core.find(idProduct=devid)
    print(usb.util.get_string(dev, dev.iSerialNumber))


def enumerate_dev() -> list:
    """Run the device enumeration and show all devices connected.
    """
    ret = []
    for dev in usb.core.find(find_all=True):
        ret.append(dev)
    return ret


def run_all_tests():
    """Run all tests in this file.
    """
    devs = enumerate_dev()
    for i in devs:
        print(i)
        # TODO finish


if __name__ == '__main__':
    run_all_tests()
