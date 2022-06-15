import cv2


def list_ports():
    """
    Test the ports and returns a tuple with the available ports and the ones that are working.
    """
    dev_port = 0
    dev_max = 500
    working_ports = []
    available_ports = []
    print('Running...')
    while dev_port < dev_max:
        camera = cv2.VideoCapture(dev_port)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if camera.isOpened():
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
        camera.release()
        dev_port +=1
    print(available_ports,working_ports)


if __name__ == "__main__":
    list_ports()
