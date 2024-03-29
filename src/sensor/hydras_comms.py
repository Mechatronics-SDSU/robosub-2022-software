
import threading, signal
from ctypes import *
from dwfconstants import *
import math
import time
import sys
import numpy as np
from queue import Queue
import scipy.signal
from pyargus.directionEstimation import *

def killHandler(signum, frame):
    dwf.FDwfDeviceCloseAll()
    exit(1)

bandpass = [-0.000175060260186362,-0.000353338686493413,-0.000621204283271244,-0.000913294948399412,-0.00114553706240163,-0.00120585043547030,-0.000977057568001650,-0.000371717537171180,0.000630425056645751,0.00195217567104058,0.00340770041020750,0.00472006189457642,0.00556872300302904,0.00566140868424901,0.00481517926079561,0.00302477021597722,0.000496844675955493,-0.00236631196611616,-0.00503843537981396,-0.00698514209788913,-0.00779859536676991,-0.00731596647625131,-0.00568600352578931,-0.00335760730670032,-0.000983256230348018,0.000745701815656647,0.00129325256812850,0.000450204097798227,-0.00156966349843214,-0.00414565447286110,-0.00638690558128367,-0.00736415875965560,-0.00638008303702647,-0.00320378205489435,0.00180313486979077,0.00771794647257308,0.0132541217427717,0.0170856038351387,0.0182176742232532,0.0163015311773594,0.0117947631140197,0.00590383954490046,0.000301044313989426,-0.00332734225808346,-0.00378997438392906,-0.000816018932120499,0.00473501997062639,0.0109653088635825,0.0153432511127505,0.0153455273271198,0.00918818137461294,-0.00354165038240891,-0.0215344885234701,-0.0418122813677345,-0.0602173067837424,-0.0722563484384914,-0.0741227594616226,-0.0636601976293061,-0.0410292676684920,-0.00889585726089716,0.0279341978598373,0.0633799979401837,0.0912743620145990,0.106617994015326,0.106617994015326,0.0912743620145990,0.0633799979401837,0.0279341978598373,-0.00889585726089716,-0.0410292676684920,-0.0636601976293061,-0.0741227594616226,-0.0722563484384914,-0.0602173067837424,-0.0418122813677345,-0.0215344885234701,-0.00354165038240891,0.00918818137461294,0.0153455273271198,0.0153432511127505,0.0109653088635825,0.00473501997062639,-0.000816018932120499,-0.00378997438392906,-0.00332734225808346,0.000301044313989426,0.00590383954490046,0.0117947631140197,0.0163015311773594,0.0182176742232532,0.0170856038351387,0.0132541217427717,0.00771794647257308,0.00180313486979077,-0.00320378205489435,-0.00638008303702647,-0.00736415875965560,-0.00638690558128367,-0.00414565447286110,-0.00156966349843214,0.000450204097798227,0.00129325256812850,0.000745701815656647,-0.000983256230348018,-0.00335760730670032,-0.00568600352578931,-0.00731596647625131,-0.00779859536676991,-0.00698514209788913,-0.00503843537981396,-0.00236631196611616,0.000496844675955493,0.00302477021597722,0.00481517926079561,0.00566140868424901,0.00556872300302904,0.00472006189457642,0.00340770041020750,0.00195217567104058,0.000630425056645751,-0.000371717537171180,-0.000977057568001650,-0.00120585043547030,-0.00114553706240163,-0.000913294948399412,-0.000621204283271244,-0.000353338686493413,-0.000175060260186362]

#import SDK
if sys.platform.startswith("win"):
    dwf = cdll.dwf
elif sys.platform.startswith("darwin"):
    dwf = cdll.LoadLibrary("/Library/Frameworks/dwf.framework/dwf")
else:
    dwf = cdll.LoadLibrary("libdwf.so")

def kill():
    dwf.FDwfAnalogOutReset(hdwf, c_int(0))
    dwf.FDwfDeviceCloseAll()
    exit(1)

def killHandler(signum, frame):
    kill()

signal.signal(signal.SIGINT, killHandler)

#declare ctype variables
target_frequency = 30e3
sensor_distance = .015
sound_speed = 1500

hdwf = c_int()
sts = c_byte()
hzAcq = c_double(500000)
switching = int(hzAcq.value/4)
fs = hzAcq.value
nSamples = int(1*hzAcq.value)
rawData1 = (c_double*nSamples)()
rawData2 = (c_double*nSamples)()
dataToSend1 = np.zeros(nSamples)
dataToSend2 = np.zeros(nSamples)
cAvailable = c_int()
cLost = c_int()
cCorrupted = c_int()
fLost = 0
fCorrupted = 0
sampleCount = 1
splineFactor = 4
splineI = np.arange(nSamples/splineFactor)
splineII = np.arange(0,nSamples/splineFactor,1.0/splineFactor)
stabilizeTime = 10


continuous = True

class function:
    """ function names """
    pulse = DwfDigitalOutTypePulse
    custom = DwfDigitalOutTypeCustom
    random = DwfDigitalOutTypeRandom
class trigger_source:
    """ trigger source names """
    none = trigsrcNone
    analog = trigsrcDetectorAnalogIn
    digital = trigsrcDetectorDigitalIn
    external = [None, trigsrcExternal1, trigsrcExternal2, trigsrcExternal3, trigsrcExternal4]

def generate(device_handle, channel, function, frequency, duty_cycle=50, data=[], wait=0, repeat=0, trigger_enabled=False, trigger_source=trigger_source.none, trigger_edge_rising=True):
    """
        generate a logic signal
        
        parameters: - channel - the selected DIO line number
                    - function - possible: pulse, custom, random
                    - frequency in Hz
                    - duty cycle in percentage, used only if function = pulse, default is 50%
                    - data list, used only if function = custom, default is empty
                    - wait time in seconds, default is 0 seconds
                    - repeat count, default is infinite (0)
                    - trigger_enabled - include/exclude trigger from repeat cycle
                    - trigger_source - possible: none, analog, digital, external[1-4]
                    - trigger_edge_rising - True means rising, False means falling, None means either, default is rising
    """
    # get internal clock frequency
    internal_frequency = c_double()
    dwf.FDwfDigitalOutInternalClockInfo(device_handle, byref(internal_frequency))
    
    # get counter value range
    counter_limit = c_uint()
    dwf.FDwfDigitalOutCounterInfo(device_handle, c_int(0), c_int(0), byref(counter_limit))
    
    # calculate the divider for the given signal frequency
    divider = int(-(-(internal_frequency.value / frequency) // counter_limit.value))
    
    # enable the respective channel
    dwf.FDwfDigitalOutEnableSet(device_handle, c_int(channel), c_int(1))
    
    # set output type
    dwf.FDwfDigitalOutTypeSet(device_handle, c_int(channel), function)
    
    # set frequency
    dwf.FDwfDigitalOutDividerSet(device_handle, c_int(channel), c_int(divider))
    
    # set wait time
    dwf.FDwfDigitalOutWaitSet(device_handle, c_double(wait))
    
    # set repeat count
    dwf.FDwfDigitalOutRepeatSet(device_handle, c_int(repeat))
    
    # enable triggering
    dwf.FDwfDigitalOutRepeatTriggerSet(device_handle, c_int(trigger_enabled))
    
    if not trigger_enabled:
        # set trigger source
        dwf.FDwfDigitalOutTriggerSourceSet(device_handle, trigger_source)
    
        # set trigger slope
        if trigger_edge_rising == True:
            # rising edge
            dwf.FDwfDigitalOutTriggerSlopeSet(device_handle, DwfTriggerSlopeRise)
        elif trigger_edge_rising == False:
            # falling edge
            dwf.FDwfDigitalOutTriggerSlopeSet(device_handle, DwfTriggerSlopeFall)
        elif trigger_edge_rising == None:
            # either edge
            dwf.FDwfDigitalOutTriggerSlopeSet(device_handle, DwfTriggerSlopeEither)

    # set PWM signal duty cycle
    if function == DwfDigitalOutTypePulse:
        # calculate counter steps to get the required frequency
        steps = int(round(internal_frequency.value / frequency / divider))
        # calculate steps for low and high parts of the period
        high_steps = int(steps * duty_cycle / 100)
        low_steps = int(steps - high_steps)
        dwf.FDwfDigitalOutCounterSet(device_handle, c_int(channel), c_int(low_steps), c_int(high_steps))
    
    # load custom signal data
    elif function == DwfDigitalOutTypeCustom:
        # format data
        buffer = (c_ubyte * ((len(data) + 7) >> 3))(0)
        for index in range(len(data)):
            if data[index] != 0:
                buffer[index >> 3] |= 1 << (index & 7)
    
        # load data
        dwf.FDwfDigitalOutDataSet(device_handle, c_int(channel), byref(buffer), c_int(len(data)))
    
    # start generating the signal
    dwf.FDwfDigitalOutConfigure(device_handle, c_int(True))
    return


def record_set_up():
    global dwf, hdwf, hzAcq, nSamples


    #print(DWF version
    version = create_string_buffer(16)
    dwf.FDwfGetVersion(version)
    print("DWF Version: "+str(version.value))

    #open device
    print("Opening first device")
    dwf.FDwfDeviceOpen(c_int(-1), byref(hdwf))

    if hdwf.value == hdwfNone.value:
        szerr = create_string_buffer(512)
        dwf.FDwfGetLastErrorMsg(szerr)
        print(str(szerr.value))
        print("failed to open device")
        quit()

    device_name = create_string_buffer(32)
    dwf.FDwfEnumDeviceName(c_int(0), device_name)
    print("First Device: " + str(device_name.value))

    


    #set up acquisition
    dwf.FDwfAnalogInBufferSizeSet(hdwf, c_int(8192)) #set buffer to 8kB (max record length = 8192/1M = )
    dwf.FDwfAnalogInChannelEnableSet(hdwf, c_int(0), c_bool(True))
    dwf.FDwfAnalogInChannelRangeSet(hdwf, c_int(0), c_double(10))
    dwf.FDwfAnalogInAcquisitionModeSet(hdwf, acqmodeRecord)
    dwf.FDwfAnalogInFrequencySet(hdwf, hzAcq)
    dwf.FDwfAnalogInRecordLengthSet(hdwf, c_double(-1)) # -1 infinite record length

    #set up pattern generation
    
    #generate(hdwf, 0, function.pulse, switching)

    # set up analog IO channel nodes
    # enable positive supply
    dwf.FDwfAnalogIOChannelNodeSet(hdwf, c_int(0), c_int(0), c_double(True)) 
    # set voltage to 5 V
    dwf.FDwfAnalogIOChannelNodeSet(hdwf, c_int(0), c_int(1), c_double(5.0)) 
    # enable negative supply
    dwf.FDwfAnalogIOChannelNodeSet(hdwf, c_int(1), c_int(0), c_double(True)) 
    # set voltage to -5 V
    dwf.FDwfAnalogIOChannelNodeSet(hdwf, c_int(1), c_int(1), c_double(-5.0)) 
    # master enable
    dwf.FDwfAnalogIOEnableSet(hdwf, c_int(True))
    print("Power supplies set to +/- 5V")


    #wait at least 2 seconds for the offset to stabilize
    time.sleep(stabilizeTime)

class Data_Record(threading.Thread):
    def __init__(self, data_process_thread, q):
        threading.Thread.__init__(self)
        self.data_process_thread = data_process_thread
        self.q = q

    def run(self):
        global dwf, hdwf, hzAcq, nSamples, rawData1, rawData2, cAvailable, cLost, cCorrupted, fLost, fCorrupted, timeElapsed

        print("Starting oscilloscope")
        dwf.FDwfAnalogInConfigure(hdwf, c_int(0), c_int(1))
        
        num = 0
        while True:
            cSamples = 0
            cLostNum = 0
            cCorrNum = 0
            times = time.time()
            while cSamples < nSamples:
                dwf.FDwfAnalogInStatus(hdwf, c_int(1), byref(sts))
                if cSamples == 0 and (sts == DwfStateConfig or sts == DwfStatePrefill or sts == DwfStateArmed) :
                    # Acquisition not yet started.
                    continue

                dwf.FDwfAnalogInStatusRecord(hdwf, byref(cAvailable), byref(cLost), byref(cCorrupted))
                
                cSamples += cLost.value

                if cLost.value :
                    fLost = 1
                    cLostNum += cLost.value
                if cCorrupted.value :
                    fCorrupted = 1
                    cCorrNum += cCorrupted.value
                if cAvailable.value==0 :
                    continue

                if cSamples+cAvailable.value > nSamples :
                    cAvailable = c_int(nSamples-cSamples)

                dwf.FDwfAnalogInStatusData(hdwf, c_int(0), byref(rawData1, sizeof(c_double)*cSamples), cAvailable) # get channel 1 data
                dwf.FDwfAnalogInStatusData(hdwf, c_int(1), byref(rawData2, sizeof(c_double)*cSamples), cAvailable) # get channel 2 data
                cSamples += cAvailable.value

            dataToSend1 = np.ctypeslib.as_array(rawData1)
            dataToSend2 = np.ctypeslib.as_array(rawData2)
            timeElapsed = time.time() - times
            if not continuous:
                num += 1
            queueData = [dataToSend1, dataToSend2, timeElapsed, cLost, cCorrupted, num]
            self.q.put(queueData)
            self.data_process_thread.event.set()
            
            if num == sampleCount and not continuous:
                break
        dwf.FDwfAnalogOutReset(hdwf, c_int(0))
        dwf.FDwfDeviceCloseAll()
        self.data_process_thread.join()

        print("Recording done")
        if fLost:
            print("Samples were lost! Reduce frequency")
        if fCorrupted:
            print("Samples could be corrupted! Reduce frequency")

        return

class Data_Process(threading.Thread):
    def __init__(self, event, q):
        threading.Thread.__init__(self)
        self.event = event
        self.q = q
        self.bigdata1 = np.empty(0)
        self.bigdata2 = np.empty(0)

                      
    def run(self):
        while True:
            print("waiting for signal")
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                client_socket.connect(('192.168.3.1', 50010))
                while True:
                    self.event.wait()

                    queueData = self.q.get()

                    procTimeElapsed = time.time()

                    num = queueData[5]
                    raw1 = queueData[0]
                    raw2 = queueData[1]
                    timeElapsed = queueData[2]
                    lostNum = queueData[3].value
                    corrNum = queueData[4].value

                    raw1b = np.convolve(raw1,bandpass,'same')
                    raw2b = np.convolve(raw2,bandpass,'same')

                    smallN = len(raw1)
                    smalln = 50
                    energies = np.zeros(int(smallN/smalln))

                    for i in range(int(smallN/smalln)):
                        x = raw1b[(i*smalln):(i+1)*smalln]
                        X = np.fft.rfft(x)
                        avg_energy = np.mean(np.absolute(X[1:3]))
                        energies[i] = avg_energy

                    n_index = np.argmax(energies>=.1)+1
                    length = 200
                    slength = 50

                    d = target_frequency*sensor_distance/sound_speed # Inter element spacing [lambda]
                    M = 2  # number of antenna elements in the antenna system (ULA)
                    N = int(length/slength)  # sample size used for the simulation


                    small1 = raw1b[n_index*smalln-1:n_index*smalln+length-1]
                    small2 = raw2b[n_index*smalln-1:n_index*smalln+length-1]

                    samples = np.zeros([int(length/slength),2],dtype = 'complex_')
                    for i in range(int(length/slength)):
                        small_fft1 = np.fft.rfft(small1[i*slength:(i+1)*slength])
                        small_fft2 = np.fft.rfft(small2[i*slength:(i+1)*slength])
                        ind = np.argmax(np.absolute(small_fft1))
                        samples[i][0] = (small_fft1[ind])/np.absolute(small_fft1[ind])
                        ind = np.argmax(np.absolute(small_fft2))
                        samples[i][1] = (small_fft2[ind])/np.absolute(small_fft2[ind])

                    final_samples =np.transpose(samples)

                    R = corr_matrix_estimate(final_samples.T, imp="mem_eff")

                    array_alignment = np.arange(0, M, 1)* d
                    incident_angles= np.arange(0,181,1)
                    ula_scanning_vectors = gen_ula_scanning_vectors(array_alignment, incident_angles)

                    MEM = np.absolute(DOA_MEM(R,ula_scanning_vectors, column_select = 1))
                    peaks = scipy.signal.find_peaks(MEM)[0]
                    if len(peaks) < 1:
                        max_peak = np.max(MEM)
                        temp_angle = np.argmax(MEM==max_peak)
                        if temp_angle == 0:
                            angle = -90
                        elif temp_angle == 180:
                            angle = 90
                        else:
                            angle = 180
                    else:
                        angle = peaks[0]-90
                    # Send to socket
                    # print(angle)
            except: 
                continue


"""
            self.bigdata1 = np.append(self.bigdata1, raw1)
            self.bigdata2 = np.append(self.bigdata2, raw2)
            if num == sampleCount and not continuous:
                print("starting to export data")
                f = open("2sensorRawDeg.csv", "w")
                for i in range(len(rawData1)):
                    f.write("%s,%s\n" % (rawData1[i],rawData2[i]))#(y1[i],y2[i],y3[i],y4[i]))
                f.close()
                print("exported data")
            
        return
"""
def data_communicate():
    while True:
        continue
    return

if __name__ == "__main__":

    time.sleep(1)
    record_set_up()

    q = Queue() #define queue to pass data between threads

    data_process_event = threading.Event()
    data_process_thread = Data_Process(data_process_event, q)
    data_process_thread.start() #set up process thead

    data_record_thread = Data_Record(data_process_thread, q)
    data_record_thread.start() #set up record thread

    data_record_thread.join() #start record thread


    #recording_thread = threading.Thread(target=data_record, args=())
    #processing_thread = threading.Thread(target=data_process, args=())
    communicating_thread = threading.Thread(target=data_communicate, args=())

    #recording_thread.start()
    #processing_thread.start()
    communicating_thread.start()

    #recording_thread.join()
    #processing_thread.join()
    communicating_thread.join()
