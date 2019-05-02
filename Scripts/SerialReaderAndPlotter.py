import serial
from pyqtgraph.Qt import QtGui, QtCore
from numpy import *
import pyqtgraph as pg
import ctypes

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Plot iterativo")
win.resize(1200, 900)
win.setWindowTitle("Heart rate Tester")

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

p1 = win.addPlot(title="Max30100 data")

win.nextRow()  # with need other plot in same window

p2 = win.addPlot(title="RED value")
p2.enableAutoRange('xy', True)
p2.setAutoPan(y=True)

heart_data = []
red_data = []

# Filter parameters
w = 0
ALPHA = 0.95
# PREV_W and PREV_W_RED are global variables
PREV_W = 0
PREV_W_RED = 0

# Mean filter parameters
MEAN_FILTER_SIZE = 15
global avg
global my_sum
global my_count
global my_index

avg = 0
my_sum = 0
my_count = 0
my_index = 0


curve1 = p1.plot(pen='y', fillLevel=-0.3, brush=(255, 0, 0, 200))
p1.setLabel('left', "Heart Rate", units=' ')
p1.setYRange(-400, 800, padding=0)

curve2 = p2.plot(pen='b')
p2.setLabel('left', "RED Data", units=' ')
p2.setYRange(-500, 500, padding=0)

ser = serial.Serial('COM7', 115200, timeout=1)  # Configure about you need

readOut = 0


def updatePlot(heart_data):
    p1.setXRange(len(heart_data)-200, len(heart_data), padding=0)



def dcRemoval(x, prev_w, alpha):
    """

    :param x: sensor last value read
    :param prev_w: last value read
    :param alpha: is the response constant of the filter
    :return: filtered value
    """
    filtered_w = x + alpha * prev_w
    result1 = filtered_w - prev_w
    global PREV_W
    PREV_W = filtered_w
    return result1

def dcRemovalRedLed(x, prev_w, alpha):
    """
    :param x: sensor last value read
    :param prev_w: last value read
    :param alpha: is the response constant of the filter
    :return: filtered value
    """
    filtered_w = x + alpha * prev_w
    result2 = filtered_w - prev_w
    global PREV_W_RED
    PREV_W_RED = filtered_w
    return result2


list_of_values = zeros(MEAN_FILTER_SIZE)  # list of data to mean median filter
bt_filtered = zeros(2)  # values of butterworthfilter


def lowPassButterworthFilter(sample):
    bt_filtered[0] = bt_filtered[1]
    bt_filtered[1] = (2.452372752527856026e-1 * sample) + (0.50952544949442879485 * bt_filtered[0])
    return bt_filtered[0] + bt_filtered[1]


def millis():
    """ return a timestamp in milliseconds (ms)"""
    tics = ctypes.c_int64()  # use *signed* 64-bit variables; see the "QuadPart" variable here: https://msdn.microsoft.com/en-us/library/windows/desktop/aa383713(v=vs.85).aspx
    freq = ctypes.c_int64()

    # get ticks on the internal ~2MHz QPC clock
    ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics))
    # get the actual freq. of the internal ~2MHz QPC clock
    ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq))

    t_ms = tics.value * 1e3 / freq.value
    return t_ms


# max value detect
last_sample = 0
count_pulse_detect = 0
max_value = 0
curve_direction = "down"
min_pulse_threshold = 200
is_reliable = 0
file = open("output.txt", 'w')
currentBeat = 0
lastBeat = 0
BPM_list = []
number_of_pulses = 0

beatDetected = False
SPO2_Flag = False
beatsDetectedNum = 0
CALCULATE_EVERY_N_BEATS = 5
irACValueSqSum = 0
redACValueSqSum = 0
samplesRecorded = 0
SPO2_list = []

spO2LUT = [100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98,
           98, 97, 97, 97, 97, 97, 97, 96, 96, 96, 96, 96, 96, 95, 95,
           95, 95, 95, 95, 94, 94, 94, 94, 94, 93, 93, 93, 93, 93]

while True:
    while ser.inWaiting() == 0:
        pass  # do nothing
    readOut = ser.readline().decode('ascii')
    # print(readOut)
    readOut = readOut.rstrip()  # remocao do \n

    # Temporary change
    splited_data = readOut.split("\t")
    # print(readOut)
    readOut = splited_data[0]
    readOutRed = splited_data[1]

    # print("IR = {}  Red = {}".format(readOut, readOutRed))

    readOut = float.fromhex(readOut)
    readOutRed = float.fromhex(readOutRed)

    # DC Removal filter
    filtered_output = dcRemoval(readOut, prev_w=PREV_W, alpha=ALPHA)
    filtered_output_red = dcRemovalRedLed(readOutRed, prev_w=PREV_W_RED, alpha=ALPHA)

    irACValue = filtered_output
    redACValue = filtered_output_red

    red_data.append(float(filtered_output_red))
    # print("IR = {}  Red = {}".format(filtered_output, filtered_output_red))

    # Mean Median Filter
    my_sum = my_sum - list_of_values[my_index]
    list_of_values[my_index] = filtered_output
    my_sum = my_sum + list_of_values[my_index]

    my_index = my_index + 1
    my_index = my_index % MEAN_FILTER_SIZE

    if my_count < MEAN_FILTER_SIZE:
        my_count = my_count + 1

    avg = my_sum / my_count

    m_filtered_output = avg - filtered_output

    # Butterworth filter
    result = lowPassButterworthFilter(float(m_filtered_output))

    heart_data.append(float(result))

    # write data in file to statical analyser
    #file.write(str(result) + " " + str(time.perf_counter()) + '\n')

    # print("IR = {}  Red = {}".format(filtered_output, filtered_output_red))

    curve1.setData(heart_data)
    curve2.setData(red_data)

    actual_sample = result
    count_pulse_detect = count_pulse_detect + 1
    if count_pulse_detect > 100:
        # print("Max count pulse detect found")
        count_pulse_detect = 0
        max_value = 0
        last_sample = 0
        actual_sample = 0
        flag_first_pulse = 0
        time_first_pulse = 0
        time_fifth_pulse = 0
    elif actual_sample > min_pulse_threshold:
        # print("actual sample = {} and last_sample = {}".format(actual_sample, last_sample))
        if actual_sample > last_sample:
            max_value = actual_sample
            curve_direction = "up"
            currentBeat = millis()
        else:
            if curve_direction == "up":
                count_pulse_detect = 0
                max_value = 0
                curve_direction = "down"
                # print("Peak reached: ")
                # print("ac_v {} las_v {}".format(actual_sample, last_sample))
                beatDuration = currentBeat - lastBeat
                lastBeat = currentBeat
                beatDetected = True
                if beatDuration > 0:
                    BPM = 60000/beatDuration
                    if 40 < BPM < 250:
                        BPM_list.append(BPM)
                        # print("BPM = {}".format(BPM))
                    if len(BPM_list) >= 5:
                        if std(BPM_list) < 5:
                            BPM_avg = sum(BPM_list) // len(BPM_list)
                            print("BPM_AVG = {}".format(BPM_avg))
                            SPO2_Flag = True
                        BPM_list.clear()

    samplesRecorded = samplesRecorded + 1
    irACValueSqSum = irACValueSqSum + irACValue*irACValue
    redACValueSqSum = redACValueSqSum + redACValue*redACValue
    if beatDetected:
        beatsDetectedNum = beatsDetectedNum + 1
        if beatsDetectedNum == CALCULATE_EVERY_N_BEATS:
            acSqRatio = 100 * log(redACValueSqSum/samplesRecorded)/log(irACValueSqSum/samplesRecorded)
            if SPO2_Flag:
                if acSqRatio > 66:
                    indice = int(acSqRatio - 66)  # variavel em portugues pra evitar sobreescrever variavel nativa do python
                    # print("index = {}".format(indice))
                spo2 = spO2LUT[indice]
                SPO2_list.append(spo2)
                if len(SPO2_list) >= 5:
                    if std(SPO2_list) < 2:
                        SPO2_avg = sum(SPO2_list) // len(SPO2_list)
                        print("SPO2 = {}".format(SPO2_avg))
                    SPO2_list.clear()
            samplesRecorded = 0
            irACValueSqSum = 0
            redACValueSqSum = 0
            beatsDetectedNum = 0
            beatDetected = False

    last_sample = result
    app.processEvents()
    updatePlot(heart_data)

    ser.flush()  # flush the buffer
