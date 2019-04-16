import serial
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.ptime import time

# app = QtGui.QApplication([])
#
# win = pg.GraphicsWindow(title="Plot iterativo")
# win.resize(1200, 900)
# win.setWindowTitle("Motion Graphics from Accelerometer Data")

# Enable antialiasing for prettier plots
# pg.setConfigOptions(antialias=True)
#
# p1 = win.addPlot(title = "Max30100 data")
# p1.enableAutoRange('xy', True)
#p1.setAutoPan(y=True)
#p1.setConfigOption('background', 'w')

# win.nextRow()  # with need other plot in same window

heart_data = []


# def updatePlot(heart_data):
#     p1.setXRange(len(heart_data)-200, len(heart_data), padding=0)
#
#
# curve1 = p1.plot(pen='y')
# p1.setLabel('left', "Heart Rate", units=' ')

#p1.setYRange(-0.2,0.2, padding = 0)
#p2.setYRange(-0.2, 0.2, padding=0)
#p3.setYRange(-0.5, 0.5, padding=0)
#p4.setYRange(-0.35, 0.35, padding=0)

ser = serial.Serial('COM26', 115200, timeout=1)  # Configure about you need

print("Starting up")

readOut = 0

while True:
    while ser.inWaiting() == 0:
        pass #do nothing
    readOut = ser.readline().decode('ascii')
    readOut = readOut.rstrip()  # remocao do \n
    print(readOut)
    #heart_data.append(float(readOut))

    #curve1.setData(heart_data)

    #app.processEvents()
    #updatePlot(heart_data)

    ser.flush()  # flush the buffer
