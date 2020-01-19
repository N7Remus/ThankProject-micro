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

class DHT_class:

    # https://github.com/adafruit/Adafruit_Python_DHT alapján
    def __init__(self, pin=17):
        self.pin = pin
        self.temp = 0
        self.hum = 0

    def printPin(self):
        print("A DHT modul a " + str(self.pin) + " pinen kommunikál")

    def update(self):
        # Érzékelő típusának beállítása : DHT11,DHT22 vagy AM2302
        # A szenzorunk a következő GPIO-ra van kötve: self.pin, ezt inicializálásko elkérem
        # Ha a read_retry eljárást használjuk. Akkor akár 15x is megpróbálja kiolvasni az érzékelőből az adatot (és minden olvasás előtt 2 másodpercet vár).
        # humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, gpio)
        humidity, temperature = Adafruit_DHT.read(Adafruit_DHT.DHT11, self.pin)
        # A DHT11 kiolvasása nagyon érzékeny az időzítésre és a Pi alkalmanként
        # nem tud helyes értéket kiolvasni. Ezért megnézzük, hogy helyesek-e a kiolvasott értékek.
        if humidity is not None and temperature is not None:
            self.temp = temperature
            self.hum = humidity

    def htmlFormat(self):
        content = "Hőmérséklet : " + str(self.temp) + " Páratartalom" + str(self.hum)
        return content


class L298N_class:

    def __init__(self, holdback=0.5,  LF_PIN=13, LB_PIN=19, RF_PIN=18, RB_PIN=12):
        GPIO.setup(LF_PIN, GPIO.OUT)  # jobbra kanyarodáshoz
        GPIO.setup(LB_PIN, GPIO.OUT)
        GPIO.setup(RF_PIN, GPIO.OUT)
        GPIO.setup(RB_PIN, GPIO.OUT)
        self.holdback = holdback
        self.pwm = [GPIO.PWM(LF_PIN, 50), GPIO.PWM(LB_PIN, 50), GPIO.PWM(RF_PIN, 50), GPIO.PWM(RB_PIN, 50)]
        '''
        for p in self.pwm:
            p.start(0)
        '''

    def changePWM(self,pin, goal):
        self.pwm[pin].ChangeDutyCycle(int(goal * self.holdback))

    def update(self, x=0, y=0):
        x = int(float(x))
        y = int(float(y))

        left = 0
        right = 0
        # balra vagy jobra megyek?
        if x < -20:
            left = abs(x)
            right = 100 - abs(x)
        elif x > 20:
            right = abs(x)
            left = 100 - abs(x)
        else:
            right = abs(y)
            left = abs(y)
        if y == 0:
            self.changePWM(0, 0)
            self.changePWM(1, 0)
            self.changePWM(2, 0)
            self.changePWM(3, 0)
        elif y < -10:
            p1 = 0
            p2 = 2
            # OFF
            p3 = 1
            p4 = 3
            self.changePWM(p3, 0)
            self.changePWM(p4, 0)
            self.changePWM(p1, left)
            self.changePWM(p2, right)
        elif y > 10:
            p1 = 1
            p2 = 3
            # OFF
            p3 = 0
            p4 = 2
            self.changePWM(p3, 0)
            self.changePWM(p4, 0)
            self.changePWM(p1, left)
            self.changePWM(p2, right)



class HCSR_class:

    # https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/ alapján
    def __init__(self, trigger=23, echo=24):
        self.trigger = trigger
        self.echo = echo
        self.distance = 0
        GPIO.setup(trigger, GPIO.OUT)
        GPIO.setup(echo, GPIO.OUT)

    def printPin(self):
        print("A modul trigger: " + str(self.trigger) + " pinen kommunikál")
        print("A modul echo: " + str(self.echo) + " pinen kommunikál")

    def update(self):
        # hc-sr04
        # ez a függvény végzi az ultrahangos szenzor működtetését.
        # kiadjuk az impulzust
        GPIO.output(self.trigger, True)
        # a jeladás után pihentetjük, majd  0.01ms után lekapcsoljuk
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)
        # kiszámoljuk az időt ami alatt a jel visszaérkezik
        StartTime = time.time()
        StopTime = time.time()
        # kezdési idő elmentése         //new
        while GPIO.input(self.echo) == 0:
            StartTime = time.time()
        # érkezési idő elmentése            //new
        while GPIO.input(self.echo) == 1:
            StopTime = time.time()

        # időkülönbség indulás és érkezés között            //new
        TimeElapsed = StopTime - StartTime
        # szorzás a szonikus sebességgel (34300 cm/s)           //new
        # és osztás kettővel az oda vissza út miatt         //new
        d = (TimeElapsed * 34300) / 2

        self.distance = int(d)

    def htmlFormat(self):
        content = "Távolság : " + str(self.distance)
        return content


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
h = HCSR_class()
h.update()
# motorvezérlő
d = DHT_class()
d.update()


l = L298N_class()
l.update()


print(m.htmlFormat())
print(d.htmlFormat())
print(h.htmlFormat())



