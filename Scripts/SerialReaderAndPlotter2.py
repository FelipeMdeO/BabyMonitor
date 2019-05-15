import serial
from pyqtgraph.Qt import QtGui, QtCore
from numpy import *
import pyqtgraph as pg
import ctypes

ser = serial.Serial('COM17', 115200, timeout=1)  # Configure about you need
IrData = 0

# plot config area
app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Max30100 data")
win.resize(1200, 900)
# win.setWindowTitle("Max30100 data")

p1 = win.addPlot(title="Heart Rate")

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

curve1 = p1.plot(pen='y', fillLevel=-0.3, brush=(255, 0, 0, 200))
p1.setLabel('left', "Heart Rate", units=' ')
p1.setYRange(-400, 800, padding=0)

heart_data = []
spo2_data = []

win.nextRow()

p2 = win.addPlot(title="SPO2 curve")
p2.setLabel('left', "SPO2", units=' ')
# p2.enableAutoRange('xy', True)
# p2.setAutoPan(y=True)
# p2.setYRange(-6000, 6000, padding=0)

curve2 = p2.plot(pen='b', fillLevel=0)

def updatePlot(data):
    p1.setXRange(len(data)-200, len(data), padding=0)

while True:
    while ser.inWaiting() == 0:
        pass

    line = ser.readline().decode('ascii')
    line = line.rstrip()

    splited_line = line.split('\t')

    if len(splited_line) < 2:
        print(line)
        continue
    else:

        irData = splited_line[0]
        redData = splited_line[1]

        # irData = float.fromhex(irData)
        # redData = float.fromhex(redData)

        # print("{}".format(irData-redData))
        print("{}   {}".format(irData, redData))
        # IrData = (int(IrData)//1000)

        # # print(IrData)
        heart_data.append(int(irData))
        spo2_data.append(int(redData))

        curve1.setData(heart_data)
        # curve2.setData(spo2_data)

        app.processEvents()
        updatePlot(heart_data)

    ser.flush()  # flush the buffer