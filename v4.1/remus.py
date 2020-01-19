# képmanipuláló modulok
from imutils.video import VideoStream
import imutils
import cv2
# webszerver
from flask import Response
from flask import Flask
from flask import render_template
from flask import request
# paraméterek, illesztőkhöz való modulok
import threading
import argparse
import datetime
import math
import time
import os.path
import Adafruit_DHT

# Raspberry specifikus modulok
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print(
        "Error importing RPi.GPIO!  Hiba az importálás közben, probáld 'sudo'-val")
import smbus  # I2C kommunkáció
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards

# pinbeállítás - mi a BCM elnevezés szerinti kiosztést használjuk
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Magnetometer

X_axis_H = 0x03  # Address of X-axis MSB data register
Z_axis_H = 0x05  # Address of Z-axis MSB data register
Y_axis_H = 0x07  # Address of Y-axis MSB data register
declination = -0.00669  # define declination angle of location where measurement going to be done
pi = 3.14159265359  # define pi value


class MPU_class:

    def __init__(self,bus ,Device_Address_MPU=0x1e # HMC5883L magnetometer device address
                 ):
        self.bus = bus
        self.Device_Address = Device_Address_MPU
        self.angle = 0
    def Magnetometer_Init(self):
        self.bus.write_byte_data(self.Device_Address, 0x37, 0x02)
        self.bus.write_byte_data(self.Device_Address, 0x6A, 0x00)
        self.bus.write_byte_data(self.Device_Address, 0x6B, 0x00)

        '''# write to Configuration Register A
        bus.write_byte_data(self.Device_Address, Register_A, 0x70)

        # Write to Configuration Register B for gain
        bus.write_byte_data(self.Device_Address, Register_B, 0xa0)

        # Write to mode Register for selecting mode
        bus.write_byte_data(self.Device_Address, Register_mode, 0)
        '''
    def read_raw_data(self,addr):
        # Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr + 1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # to get signed value from mpu6050
        if value > 32768:
            value = value - 65536
        return value

    def printPin(self):
        print("Az iránytű modul modul: " + str(self.Device_Address) + " címen kommunikál")

    def update(self):
        print("update")
        # magnetométer infója
        # Read Accelerometer raw value
        x = self.read_raw_data(X_axis_H)
        z = self.read_raw_data(Z_axis_H)
        y = self.read_raw_data(Y_axis_H)

        #print(x, y, z)

        heading = math.atan2(y, x) + declination

        # Due to declination check for >360 degree
        if heading > 2 * pi:
            heading = heading - 2 * pi

        # check for sign
        if heading < 0:
            heading = heading + 2 * pi

        # convert into angle
        heading_angle = int(heading * 180 / pi)
        self.angle = heading_angle

    def htmlFormat(self):
        content = "Szög : " + str(self.angle)
        return content


m = MPU_class(bus)
m.update()
print(m.htmlFormat())
