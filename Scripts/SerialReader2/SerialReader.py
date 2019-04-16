import serial
import struct
import time
import csv

ser = serial.Serial('COM26', 115200, timeout=1)
ser.flushInput()

print("Starting !")

# TODO implementar variavel no c que envia o numero da amostra para que se tenha perceba que nao esta se perdendo amostras

while True:
    try:
        ser_bytes = ser.readline()
        #decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        decoded_bytes = ser_bytes[0:len(ser_bytes) - 2].decode("utf-8")
        print(decoded_bytes)
    except:
        print("Keyboard Interrupt")
        break