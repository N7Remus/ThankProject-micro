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

# pinbeállítás
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# CONTROLS
# LEFT_FORWARD_PIN - lf
LF_PIN = 13
LB_PIN = 19
RF_PIN = 18
RB_PIN = 12

GPIO.setup(LF_PIN, GPIO.OUT)  # jobbra kanyarodáshoz
GPIO.setup(LB_PIN, GPIO.OUT)
GPIO.setup(RF_PIN, GPIO.OUT)
GPIO.setup(RB_PIN, GPIO.OUT)

# Ultrasonic
GPIO_TRIGGER = 23
GPIO_ECHO = 24
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
# publikus változó a távolság meghatározásához
dist = 0
# hőmérséklet
temp = 0
# páratartalom
hum = 0

# a pwm pinek, melyekkel a motrok vezérelve lesznek az alábbi tömbben lesznek letárolva
pwm = [GPIO.PWM(LF_PIN, 50), GPIO.PWM(LB_PIN, 50), GPIO.PWM(RF_PIN, 50), GPIO.PWM(RB_PIN, 50)]


def distance():
    # hc-sr04
    # ez a függvény végzi az ultrahangos szenzor működtetését.
    global dist
    # kiadjuk az impulzust
    GPIO.output(GPIO_TRIGGER, True)

    # a jeladás után pihentetjük, majd  0.01ms után lekapcsoljuk
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    # kiszámoljuk az időt ami alatt a jel visszaérkezik
    StartTime = time.time()
    StopTime = time.time()
    # kezdési idő elmentése         //new
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
    # érkezési idő elmentése            //new
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # időkülönbség indulás és érkezés között            //new
    TimeElapsed = StopTime - StartTime
    # szorzás a szonikus sebességgel (34300 cm/s)           //new
    # és osztás kettővel az oda vissza út miatt         //new
    d = (TimeElapsed * 34300) / 2

    dist = int(d)


def DHT11_read():
    # Érzékelő típusának beállítása : DHT11,DHT22 vagy AM2302
    # A szenzorunk a következő GPIO-ra van kötve:
    global temp, hum
    gpio = 17

    # Ha a read_retry eljárást használjuk. Akkor akár 15x is megpróbálja kiolvasni az érzékelőből az adatot (és minden olvasás előtt 2 másodpercet vár).
    # humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, gpio)
    humidity, temperature = Adafruit_DHT.read(Adafruit_DHT.DHT11, gpio)
    # A DHT11 kiolvasása nagyon érzékeny az időzítésre és a Pi alkalmanként
    # nem tud helyes értéket kiolvasni. Ezért megnézzük, hogy helyesek-e a kiolvasott értékek.
    if humidity is not None and temperature is not None:
        temp = temperature
        hum = humidity
    else:
        return "", ""


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

# some MPU6050 Registers and their Address
Register_A = 0  # Address of Configuration register A
Register_B = 0x01  # Address of configuration register B
Register_mode = 0x02  # Address of mode register

X_axis_H = 0x03  # Address of X-axis MSB data register
Z_axis_H = 0x05  # Address of Z-axis MSB data register
Y_axis_H = 0x07  # Address of Y-axis MSB data register
declination = -0.00669  # define declination angle of location where measurement going to be done
pi = 3.14159265359  # define pi value


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


def Magnetometer_Init():
    bus.write_byte_data(Device_Address_MPU, 0x37, 0x02)
    bus.write_byte_data(Device_Address_MPU, 0x6A, 0x00)
    bus.write_byte_data(Device_Address_MPU, 0x6B, 0x00)

    # write to Configuration Register A
    bus.write_byte_data(Device_Address, Register_A, 0x70)

    # Write to Configuration Register B for gain
    bus.write_byte_data(Device_Address, Register_B, 0xa0)

    # Write to mode Register for selecting mode
    bus.write_byte_data(Device_Address, Register_mode, 0)


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
    # A gyorsulásmérő nyers adatainak beolvasása            //new
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    # A giroszkóp nyers adatainak beolvasása            //new
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    # A teljes mérési tartománya +- 250 °C érzékenységi skála tényezőnként          //new
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0
    print("Gx=%.2f" % Gx, u'\u00b0' + "/s", "\tGy=%.2f" % Gy, u'\u00b0' + "/s", "\tGz=%.2f" % Gz, u'\u00b0' + "/s",
          "\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az)


def angle():
    # magnetométer infója
    # Read Accelerometer raw value
    x = read_raw_data(X_axis_H)
    z = read_raw_data(Z_axis_H)
    y = read_raw_data(Y_axis_H)

    print(x, y, z)

    heading = math.atan2(y, x) + declination

    # Due to declination check for >360 degree
    if heading > 2 * pi:
        heading = heading - 2 * pi

    # check for sign
    if heading < 0:
        heading = heading + 2 * pi

    # convert into angle
    heading_angle = int(heading * 180 / pi)
    return heading_angle


bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address
MPU_Init()


class mySensors(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.board = 1

    def run(self):
        while True:
            #distance()
            #gyro()
            time.sleep(1)
            #DHT11_read()
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
# új képkocka létrehozása és zárolása amíg az első folyamatban van          //new
outputFrame = None
lock = threading.Lock()

# initialize a flask object
# flask objektum létrehozása         //new
app = Flask(__name__)

# initialize the video stream and allow the camera sensor to
# warmup
# vs = VideoStream(usePiCamera=1).start()
# video közvetítés létrehozása és kamera szenzor engedélyezése           /new
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
    # visszaadja a render sablont           //new
    return render_template("basic.html")
    #basic2 -kamerateszt

def detect_motion():
    # grab global references to the video stream, output frame, and
    # lock variables
    # a globális referenciákat beleteszi a video közvetítésbe, kimeneti képkockába és a zároló változóba            //new
    global vs, outputFrame, lock

    # loop over frames from the video stream
    # kiolvassa a következő képkockát, majd újra méretezi, beszürkíti és elhomályosítja a képkockát         //new
    while True:
        # read the next frame from the video stream, resize it,
        # convert the frame to grayscale, and blur it
        frame = vs.read()
        #frame = imutils.resize(frame, width=400)

        with lock:
            outputFrame = frame.copy()


def generate():
    # grab global references to the output frame and lock variables
    # hozzáadja a globális referenciát a kimeneti képkockához és zárolja a változókat           //new
    global outputFrame, lock

    # loop over frames from the output stream
    # várakozik amíg a zároló meg nem kapta, ellenőrzi hogy elérhető-e a kimeneti képkocka, ellenkező esetben továbbugrik           //new
    while True:
        # wait until the lock is acquired
        with lock:
            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            if outputFrame is None:
                continue

            # encode the frame in JPEG format
            # képkocka kódolása JPEG formátumban            //new
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)

            # ensure the frame was successfully encoded
            # ellenőrzi hogy sikeres volt-e a kódolás           //new
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
    # SR04 (Ultrahangos távolságszenzor) adatai itt van egy javítás, mivel a szenzor 4-9 cm-t csal
    content += "<h2>Távolság : " + str(dist) + " Valós: " + str(dist - 9) + "</h2>"
    # MPU 6050
    content += "<h2>Gyorsulás : " + str(Ax) + " " + str(Ay) + " " + str(Az) + "</h2>"
    content += "<h2>Helyzet : " + str(Gx) + " " + str(Gy) + " " + str(Gz) + "</h2>"
    # HMC5883L
    #content += "<h2>Angle : " + str(angle()) + "</h2>"

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
    args = vars(ap.parse_args())

    # start a thread that will perform motion detection
    # mozgásérzékelés indítása          //new

    t = threading.Thread(target=detect_motion, args=())
    t.daemon = True
    t.start()

    # sensors.start()

    # start the flask app
    # flask alkalmazás indítása         //new

    app.run(host=args["ip"], port=args["port"], debug=True,
            threaded=True, use_reloader=False)

# release the video stream pointer
vs.stop()

for p in pwm:
    p.stop()

GPIO.cleanup()
