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

#Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

p1 = win.addPlot(title = "Max30100 data")
p1.enableAutoRange('xy', True)
p1.setAutoPan(y=True)

#p1.setConfigOption('background', 'w')

win.nextRow()  # with need other plot in same window

heart_data = []

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


list_of_values = zeros(MEAN_FILTER_SIZE)  #list of data to mean median filter

while True:
    while ser.inWaiting() == 0:
        pass #do nothing
    readOut = ser.readline().decode('ascii')
    #print(readOut)
    readOut = readOut.rstrip()  # remocao do \n
    readOut = float.fromhex(readOut)

    filtered_output = dcRemoval(readOut, prev_w=PREV_W, alpha=ALPHA)

    my_sum = my_sum - list_of_values[my_index]
    list_of_values[my_index] = filtered_output
    my_sum = my_sum + list_of_values[my_index]

    my_index = my_index + 1
    my_index = my_index % MEAN_FILTER_SIZE

    if my_count < MEAN_FILTER_SIZE:
        my_count = my_count + 1

    avg = my_sum / my_count

    m_filtered_output = avg - filtered_output

    heart_data.append(float(m_filtered_output))

    curve1.setData(heart_data)

    app.processEvents()
    updatePlot(heart_data)

    ser.flush()  # flush the buffer
