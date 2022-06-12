import subprocess

f = open('start/current_devices.cfg', 'r')
for lines in f:
    subprocess.run(['sudo', 'chmod', '777', f'/dev/{lines.strip()}'])
f.close()
subprocess.run(['sudo', 'chmod', '777', '/dev/ttyACM0'])
