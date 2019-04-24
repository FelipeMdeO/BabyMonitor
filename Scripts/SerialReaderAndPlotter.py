import serial
from pyqtgraph.Qt import QtGui, QtCore
from numpy import *
import pyqtgraph as pg
from pyqtgraph.ptime import time
import time

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Plot iterativo")
win.resize(1200, 900)
win.setWindowTitle("Heart rate Tester")

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

p1 = win.addPlot(title="Max30100 data")
# p1.enableAutoRange('xy', True)
# p1.setAutoPan(y=True)

# p1.setConfigOption('background', 'w')

win.nextRow()  # with need other plot in same window

# p2 = win.addPlot(title="Max value")
# p2.enableAutoRange('xy', True)
# p2.setAutoPan(y=True)


heart_data = []
# max_pulse = []

# Filter parameters
w = 0
global PREV_W
PREV_W = 0
ALPHA = 0.95

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

# curve2 = p2.plot(pen=None, symbol='t', symbolPen=None, symbolSize=10, symbolBrush=(100, 100, 255, 50))
# p2.setLabel('left', "Max value", units=' ')
# p2.setYRange(-400, 800, padding=0)

ser = serial.Serial('COM7', 115200, timeout=1)  # Configure about you need

readOut = 0

def updatePlot(heart_data):
    p1.setXRange(len(heart_data)-200, len(heart_data), padding=0)


def dcRemoval(x, prev_w, alpha):
    filtered_w = x + alpha * prev_w
    result = filtered_w - prev_w
    global PREV_W
    PREV_W = filtered_w
    return result


list_of_values = zeros(MEAN_FILTER_SIZE)  # list of data to mean median filter
bt_filtered = zeros(2)  # values of butterworthfilter


def lowPassButterworthFilter(sample):
    bt_filtered[0] = bt_filtered[1]
    bt_filtered[1] = (2.452372752527856026e-1 * sample) + (0.50952544949442879485 * bt_filtered[0])
    return bt_filtered[0] + bt_filtered[1]


# max value detect
last_sample = 0
count_pulse_detect = 0
max_value = 0
curve_direction = "down"
time_first_pulse = 0
time_fifth_pulse = 0
flag_first_pulse = 0
number_of_pulses = 0
n_pulses_to_count = 10
min_pulse_threshold = 100
is_reliable = 0

while True:
    while ser.inWaiting() == 0:
        pass # do nothing
    readOut = ser.readline().decode('ascii')
    # print(readOut)
    readOut = readOut.rstrip()  # remocao do \n
    readOut = float.fromhex(readOut)

    # DC Removal filter
    filtered_output = dcRemoval(readOut, prev_w=PREV_W, alpha=ALPHA)

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
    # heart_data.append(float(m_filtered_output))

    curve1.setData(heart_data)
    actual_sample = result
    count_pulse_detect = count_pulse_detect + 1
    if count_pulse_detect > 100:
        print("Max count pulse detect found")
        count_pulse_detect = 0
        max_value = 0
        last_sample = 0
        actual_sample = 0
        flag_first_pulse = 0
        time_first_pulse = 0
        time_fifth_pulse = 0
        number_of_pulses = 0
    elif actual_sample > min_pulse_threshold:
        # print("actual sample = {} and last_sample = {}".format(actual_sample, last_sample))
        if actual_sample > last_sample:
            max_value = actual_sample
            curve_direction = "up"
        else:
            if curve_direction == "up":
                # print("max_value = {}".format(max_value))
                # max_pulse.append(float(max_value))
                number_of_pulses = number_of_pulses + 1
                # curve2.setData(max_pulse)
                # p2.plot(max_pulse, pen=(0, 0, 255))
                count_pulse_detect = 0
                max_value = 0
                last_sample = 0
                actual_sample = 0
                curve_direction = "down"
    if number_of_pulses == 1:  # when I find first pulse save the time for it
        time_first_pulse = time.perf_counter()
        flag_first_pulse = 1
    if flag_first_pulse and number_of_pulses == n_pulses_to_count+1:  # when I find the pulse of number 5 save time for it
        time_fifth_pulse = time.perf_counter()
        dif_time = time_fifth_pulse - time_first_pulse  # verify diff time
        if is_reliable:
            print("BPM = {}".format(60 * n_pulses_to_count//dif_time))  # if 5 bp in dif time, how much bp in 60 seconds ? result = 5*60/diff
        else:
            is_reliable = 1
        flag_first_pulse = 0
        time_first_pulse = 0
        time_fifth_pulse = 0
        number_of_pulses = 0

    last_sample = result
    app.processEvents()
    updatePlot(heart_data)

    ser.flush()  # flush the buffer
