#!/usr/bin/env python
import time
import serial 

Ard = serial.Serial('/dev/ttyUSB0',115200)
time.sleep(1)

def heading():
    dataPacket=Ard.readline()
    dataPacket = str(dataPacket)
    data_f = float(dataPacket)
    return data_f

while True :
    dataPacket = Ard.readline()
    print(heading())
