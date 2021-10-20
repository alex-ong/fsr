"""
Serial gui for showing current
floating point values for the pad
you can also send a magic packet
back to the pad for it to re-write it's
eeprom so you can store pad sensitivity inside
"""

# try to import pyserial

try:
    import serial
except ImportError:
    print("Please check readme.md for installation and running instructions!")
    input("Press enter to continue...")
    import sys

    sys.exit()

from serial.tools import list_ports
import tkinter as tk
import sys
import time
import json

PORT = "COM4"
ENDIAN = "big"


def get_ports():
    """
    returns ["COM3", "COM4" ...]
    """
    items = list(list_ports.comports())
    for item in items:
        print(item.device)
        return [item.device for item in items]


class SerialPort:
    def __init__(self):
        self.port = None

    def connect(self):
        print("attempting to connect to ", PORT)
        self.port = serial.Serial(PORT, 115200)
        print("connected!")

    def disconnect(self):
        self.port.close()
        self.port = None

    def read(self):
        if self.port is None:
            return []
        lines = []
        lines_read = 0
        while self.port.in_waiting and lines_read < 100:
            data = self.port.read_until()
            lines.append(data)
            lines_read += 1
        return lines

    def port_write_led(self):
        values = b'l'
        self.port.write(values)
        
if __name__ == "__main__":
    sp = SerialPort()
    sp.connect()
    sp.port_write_led();
    print('written')
