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
'''
from DHT11_sens import DHT_class
from HC-SR04_sens import HCSR_class
from L298N_sens import L298N_class
'''
from MPU_sens import MPU_class
m = MPU_class()
m.update()
print(m.htmlForm())
