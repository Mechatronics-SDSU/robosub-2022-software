
import fileinput
import re


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
                    parsed_test_str = parse_tty(test)
                    if parsed_test_str not in written_devs:
                        written_devs.append(parsed_test_str)
                        f.write(f'{parsed_test_str}\n')


if __name__ == '__main__':
    match_from_port()
