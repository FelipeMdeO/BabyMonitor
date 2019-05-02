import serial
from pyqtgraph.Qt import QtGui, QtCore
from numpy import *
import pyqtgraph as pg
import ctypes

ser = serial.Serial('COM7', 115200, timeout=1)  # Configure about you need
IrData = 0

# plot config area
app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Plot iterativo")
win.resize(1200, 900)
win.setWindowTitle("Heart rate Tester")

p1 = win.addPlot(title="Max30100 data")

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

curve1 = p1.plot(pen='y', fillLevel=-0.3, brush=(255, 0, 0, 200))
p1.setLabel('left', "Heart Rate", units=' ')
p1.setYRange(-400, 800, padding=0)

heart_data = []


def updatePlot(data):
    p1.setXRange(len(data)-200, len(data), padding=0)

while True:
    while ser.inWaiting() == 0:
        pass
    IrData = ser.readline().decode('ascii')
    IrData = IrData.rstrip()
    IrData = (int(IrData)//1000)
    #print(readOut)
    heart_data.append(IrData)

    curve1.setData(heart_data)
    app.processEvents()
    updatePlot(heart_data)
    ser.flush()  # flush the buffer