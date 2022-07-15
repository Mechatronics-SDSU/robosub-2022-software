
import fileinput
import re
import os


pat_1 = 'ttyUSB'


def get_devs_from_file() -> list:
    ret = []
    f = open('start/current_devices.cfg', 'r')
    for line in f:
        ret.append(line.strip())
    return ret


def parse_tty(in_str: str) -> str:
    t_1 = 0
    ti = 0
    t_2 = 0
    for i in range(len(in_str)):
        if (in_str[i] == 't') and (t_1 == 0):
            t_1 = 1
            ti = i
        elif (in_str[i] == 't') and (t_2 == 0):
            if ti == i - 1:
                t_2 = 1
            else:
                t_1 = 0
        elif (in_str[i] == 'y') and (t_2 == 1):
            return in_str[ti:].strip()


def match_from_port() -> None:
    devs = get_devs_from_file()
    written_devs = []
    f = open('start/current_devices.cfg', 'w')
    f_1 = open('start/current_device_names.cfg', 'r')
    f_1_entries = f_1.readlines()
    f_1.close()
    for name in f_1_entries:
        f_1_entries[f_1_entries.index(name)] = f_1_entries[f_1_entries.index(name)].rstrip('\n')
    ind = 0
    dev_names = []
    for i in range(len(devs)):
        dev_names.append('')
    for line in fileinput.input():
        for dev in devs:
            line = line.strip()
            pat = re.compile(dev)
            test = re.search(pattern=pat, string=line)
            if test is not None:
                test = test.string
                test = re.search(pattern=pat_1, string=test)
                if test is not None:
                    test = test.string
                    dev_names[ind] = f_1_entries[devs.index(dev)]
                    parsed_test_str = parse_tty(test)
                    if parsed_test_str not in written_devs:
                        written_devs.append(parsed_test_str)
                        f.write(f'/dev/{parsed_test_str}\n')
                        ind += 1
    os.remove('start/current_device_names.cfg')
    f_1 = open('start/current_device_names.cfg', 'w+')
    for dev_name in dev_names:
        f_1.write(f'{dev_name}\n')
    f_1.close()


if __name__ == '__main__':
    match_from_port()
