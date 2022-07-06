from brping import Ping1D
import time

class sonarDriver():
    """
    Attempt to create a serial connection, a get state method invoked to get the 
    distance that sonar currently has 
    """
    def __init__(self) -> None: 
        port = "/dev/cu.usbserial-D2009U7G"
        myPing.connect_serial(port, 115200)

        if myPing.initialize() is False:
            print('Failed to initialize Ping!')
            exit(1)
            
        print("Starting Ping..")
    
    def getstate(self, myPing):
        data = myPing.get_distance()
        if data:
            print("Distance: %s\tConfidence: %s%%" % (data["distance"], data["confidence"]))
        else:
            print("Failed to get distance data")
        time.sleep(0.1)

if __name__=='__main__':
    myPing = Ping1D()
    sonar = sonarDriver()

    while True:
        sonar.getstate(myPing)

    