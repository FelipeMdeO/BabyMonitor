import serial
from pyqtgraph.Qt import QtGui, QtCore
from numpy import *
import pyqtgraph as pg

ser = serial.Serial('COM20', 115200, timeout=1)  # Configure about you need
IrData = 0

# plot config area
app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Max30100 data")
win.resize(1200, 900)

p1 = win.addPlot(title="Heart Rate")

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

curve1 = p1.plot(pen='y', fillLevel=-0.3, brush=(255, 0, 0, 200))
p1.setLabel('left', "Heart Rate", units=' ')
p1.setYRange(-800, 800, padding=0)

heart_data = []

win.nextRow()

p2 = win.addPlot(title="SPO2 curve")
p2.setLabel('left', "SPO2", units=' ')

curve2 = p2.plot(pen='b', fillLevel=0)

# Filter parameters
w = 0
ALPHA = 0.95
# PREV_W and PREV_W_RED are global variables
PREV_W = 0

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

def updatePlot(data):
    p1.setXRange(len(data)-200, len(data), padding=0)

while True:
    while ser.inWaiting() == 0:
        pass
    line = ser.readline().decode('ascii')
    line = line.rstrip()
    try:
        irData = int(line.split(';')[0])
        print("{}".format(irData))
        # print(line)
        # DC Removal filter
        filtered_output = float(dcRemoval(float(irData), prev_w=PREV_W, alpha=ALPHA))
        irACValue = filtered_output

        heart_data.append(irACValue)
        curve1.setData(heart_data)
        app.processEvents()
        updatePlot(heart_data)
    except:
        continue
    ser.flush()  # flush the buffer