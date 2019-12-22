# coding=utf-8
# USAGE
# python webstreaming.py --ip 0.0.0.0 --port 8000

# import the necessary packages
from imutils.video import VideoStream
from flask import Response
from flask import Flask
from flask import render_template
from flask import request
import threading
import argparse
import datetime
import imutils
import math
import time
import cv2
import os.path
import Adafruit_DHT

# Raspberry specifikus modulok
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print(
        "Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")
import smbus  # import SMBus module of I2C

# https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/ example alajpján
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# CONTROLS
# LEFT_FORWARD_PIN - lf
LF_PIN = 13
LB_PIN = 19
RF_PIN = 18
RB_PIN = 12

# Ultrasonic
GPIO_TRIGGER = 23
GPIO_ECHO = 24

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
# ultrasonic szenzor
dist = 0
# hőmérséklet
temp = 0
# páratartalom
hum = 0

pwm = []

GPIO.setup(LF_PIN, GPIO.OUT)  # jobbra kanyarodáshoz
GPIO.setup(LB_PIN, GPIO.OUT)
GPIO.setup(RF_PIN, GPIO.OUT)
GPIO.setup(RB_PIN, GPIO.OUT)

pwm.append(GPIO.PWM(LF_PIN, 50))
pwm.append(GPIO.PWM(LB_PIN, 50))
pwm.append(GPIO.PWM(RF_PIN, 50))
pwm.append(GPIO.PWM(RB_PIN, 50))


# wait_for_ajax = 100 # x ajax kérés után kárjök le a dht szenzort, mivel egyébként lassítja
def distance():
    global dist
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    dist = int(distance)


def DHT11_read():
    # Érzékelő típusának beállítása : DHT11,DHT22 vagy AM2302
    # A szenzorunk a következő GPIO-ra van kötve:
    global temp, hum
    gpio = 17

    # A read_retry eljárást használjuk. Ez akár 15x is megpróbálja kiolvasni az érzékelőből az adatot (és minden olvasás előtt 2 másodpercet vár).
    # humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, gpio)
    humidity, temperature = Adafruit_DHT.read(Adafruit_DHT.DHT11, gpio)
    # A DHT11 kiolvasása nagyon érzékeny az időzítésre és a Pi alkalmanként
    # nem tud helyes értéket kiolvasni. Ezért megnézzük, hogy helyesek-e a kiolvasott értékek.
    if humidity is not None and temperature is not None:
        temp = temperature
        hum = humidity
    else:
        return "", ""


# some MPU6050 Registers and their Address
Device_Address_MPU = 0x1e  # HMC5883L magnetometer device address

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

Ax = 0
Ay = 0
Az = 0

Gx = 0
Gy = 0
Gz = 0


def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if (value > 32768):
        value = value - 65536
    return value


def gyro():
    global Ax, Ay, Az, Gx, Gy, Gz
    # Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    # Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0
    print("Gx=%.2f" % Gx, u'\u00b0' + "/s", "\tGy=%.2f" % Gy, u'\u00b0' + "/s", "\tGz=%.2f" % Gz, u'\u00b0' + "/s",
          "\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az)


bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address
MPU_Init()


class mySensors(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.board = 1

    def run(self):
        while True:
            distance()
            gyro()
            time.sleep(1)
            DHT11_read()
            time.sleep(1)
        pass


sensors = mySensors()


def changePWM(pin, goal):
    global pwm
    holdback = 0.5

    print("Pin: ", pin, "Freq: ", int(goal * holdback))

    pwm[pin].start(0)
    pwm[pin].ChangeDutyCycle(int(goal * holdback))


# USB CAMERA

# initialize the output frame and a lock used to ensure thread-safe
# exchanges of the output frames (useful for multiple browsers/tabs
# are viewing tthe stream)
outputFrame = None
lock = threading.Lock()

# initialize a flask object
app = Flask(__name__)

# initialize the video stream and allow the camera sensor to
# warmup
# vs = VideoStream(usePiCamera=1).start()
vs = VideoStream(src=0).start()
time.sleep(2.0)


def root_dir():  # pragma: no cover
    return os.path.abspath(os.path.dirname(__file__))


def get_file(filename):  # pragma: no cover
    try:
        src = os.path.join(root_dir(), filename)
        # Figure out how flask returns static files
        # Tried:
        # - render_template
        # - send_file
        # This should not be so non-obvious
        return open(src).read()
    except IOError as exc:
        return str(exc)


@app.route("/")
def index():
    # return the rendered template
    # return render_template("index.html")
    # return render_template("joystick.html")
    return render_template("basic.html")


def detect_motion(frameCount):
    # grab global references to the video stream, output frame, and
    # lock variables
    global vs, outputFrame, lock

    # loop over frames from the video stream
    while True:
        # read the next frame from the video stream, resize it,
        # convert the frame to grayscale, and blur it
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        with lock:
            outputFrame = frame.copy()


def generate():
    # grab global references to the output frame and lock variables
    global outputFrame, lock

    # loop over frames from the output stream
    while True:
        # wait until the lock is acquired
        with lock:
            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            if outputFrame is None:
                continue

            # encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)

            # ensure the frame was successfully encoded
            if not flag:
                continue

        # yield the output frame in the byte format
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encodedImage) + b'\r\n')


@app.route("/video_feed")
def video_feed():
    # return the response generated along with the specific media
    # type (mime type)
    return Response(generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/ajax/")
def ajax():
    global Ax, Ay, Az, Gx, Gy, Gz

    # here we want to get the value of user (i.e. ?user=some-value)
    x = request.args.get('w')
    y = request.args.get('h')
    x = int(float(x))
    y = int(float(y))
    left = 0
    right = 0

    #balra vagy jobra megyek?
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
        changePWM(0, 0)
        changePWM(1, 0)
        changePWM(2, 0)
        changePWM(3, 0)
    elif y < -10:
        p1 = 0
        p2 = 2
        # OFF
        p3 = 1
        p4 = 3
        changePWM(p3, 0)
        changePWM(p4, 0)
        changePWM(p1, left)
        changePWM(p2, right)
    elif y > 10:
        p1 = 1
        p2 = 3
        # OFF
        p3 = 0
        p4 = 2
        changePWM(p3, 0)
        changePWM(p4, 0)
        changePWM(p1, left)
        changePWM(p2, right)

    # Inputok kijelzése
    content = "<h2>Inputok(x/y) : " + str(x) + " : " + str(y) + "</h2>"
    # DHT11 (Hőfok és páratartalom szenzor) adatai
    content += "<h2>Páratartalom / Hőmérséklet : " + str(hum) + " ; " + str(temp) + "</h2>"
    # SR04 (Ultrahangos távolságszenzor) adatai
    content += "<h2>Távolság : " + str(dist) + " Valós: " + str(dist - 9) + "</h2>"
    # MPU 6050
    content += "<h2>Gyorsulás : " + str(Ax) + " " + str(Ay) + " " + str(Az) + "</h2>"
    content += "<h2>Helyzet : " + str(Gx) + " " + str(Gy) + " " + str(Gz) + "</h2>"
    # HMC5883L
    # content += "<h2>Angle : " + str(angle()) + "</h2>"

    mimetype = "text/html"

    return Response(content, mimetype=mimetype)


@app.route('/<path:path>')
def get_resource(path):  # pragma: no cover
    mimetypes = {
        ".css": "text/css",
        ".html": "text/html",
        ".js": "application/javascript",
    }
    complete_path = os.path.join(root_dir(), path)
    ext = os.path.splitext(path)[1]
    mimetype = mimetypes.get(ext, "text/html")
    content = get_file(complete_path)
    return Response(content, mimetype=mimetype)


# check to see if this is the main thread of execution
if __name__ == '__main__':
    # construct the argument parser and parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--ip", type=str, required=True,
                    help="ip address of the device")
    ap.add_argument("-o", "--port", type=int, required=True,
                    help="ephemeral port number of the server (1024 to 65535)")
    ap.add_argument("-f", "--frame-count", type=int, default=32,
                    help="# of frames used to construct the background model")
    args = vars(ap.parse_args())

    # start a thread that will perform motion detection
    t = threading.Thread(target=detect_motion, args=(
        args["frame_count"],))
    t.daemon = True
    t.start()

    # start the flask app
    sensors.start()
    app.run(host=args["ip"], port=args["port"], debug=True,
            threaded=True, use_reloader=False)

# release the video stream pointer
vs.stop()

for p in pwm:
    p.stop()

GPIO.cleanup()
