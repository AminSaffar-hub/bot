import time
import serial 

arduinoData=serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AC008RXB-if00-port0',115200)
dataPacket=arduinoData.readline()
dataPacket=arduinoData.readline()


def heading():
    dataPacket=arduinoData.readline()
    dataPacket = str(dataPacket)
    dataPacket = float(dataPacket[2:-5])
    return dataPacket

while True :
    dataPacket=arduinoData.readline()

    print(heading())
