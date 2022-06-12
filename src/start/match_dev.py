
import fileinput
import re


def known_dev_enum() -> list:
    known_devs = []
    f = open('start/known_devices.cfg', 'r')
    f_1 = open('start/known_device_names.cfg', 'r')
    for line in f:
        ret = line.strip()
        if ret is not None:
            known_devs.append([ret.strip(), f_1.readline().strip()])
    f.close()
    f_1.close()
    return known_devs


def parse_loc(in_str: str) -> str:
    ind_1 = -1
    ind_2 = -1
    for i in range(len(in_str)):
        if in_str[i] == '[':  # Timestamp
            ind_1 = i
        elif (in_str[i] == ']') and (ind_1 != -1):
            ind_2 = i+2
        elif in_str[i] == ':' and (ind_2 != -1):  # End of dev
            return f'{in_str[ind_2:i]}\n'


def match_devices() -> None:
    known_devs = known_dev_enum()
    written_devs = []
    f = open('start/current_devices.cfg', 'w')
    f_1 = open('start/current_device_names.cfg', 'w')
    for line in fileinput.input():
        for dev in known_devs:
            line = line.strip()
            pat = re.compile(dev[0])
            test = re.search(pattern=pat, string=line)
            if test is not None:
                found_dev = parse_loc(test.string)
                if found_dev not in written_devs:
                    written_devs.append(found_dev)
                    f.write(found_dev)
                    f_1.write(f'{dev[1]}\n')
    f.close()


if __name__ == '__main__':
    match_devices()
