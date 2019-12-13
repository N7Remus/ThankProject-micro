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
import time
import cv2
import os.path
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

#Magnetométer
# some MPU6050 Registers and their Address
Register_A = 0  # Address of Configuration register A
Register_B = 0x01  # Address of configuration register B
Register_mode = 0x02  # Address of mode register

X_axis_H = 0x03  # Address of X-axis MSB data register
Z_axis_H = 0x05  # Address of Z-axis MSB data register
Y_axis_H = 0x07  # Address of Y-axis MSB data register
declination = -0.00669  # define declination angle of location where measurement going to be done
pi = 3.14159265359  # define pi value

address = 0x68

# Gyro

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c


def Magnetometer_Init():
    bus.write_byte_data(address, 0x37, 0x02)
    bus.write_byte_data(address, 0x6A, 0x00)
    bus.write_byte_data(address, 0x6B, 0x00)

    # write to Configuration Register A
    bus.write_byte_data(Device_Address, Register_A, 0x70)

    # Write to Configuration Register B for gain
    bus.write_byte_data(Device_Address, Register_B, 0xa0)

    # Write to mode Register for selecting mode
    bus.write_byte_data(Device_Address, Register_mode, 0)


def read_raw_data(addr):
    # Read raw 16-bit value
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from module
    if (value > 32768):
        value = value - 65536
    return value


def read_byte(reg):
    return bus.read_byte_data(address, reg)


def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg + 1)
    value = (h << 8) + l
    return value


def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val


def dist(a, b):
    return math.sqrt((a * a) + (b * b))


def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)


def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)


bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x1e  # HMC5883L magnetometer device address
g_address = 0x68       #  Gyro

# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)

Magnetometer_Init()  # initialize HMC5883L magnetometer

# CONTROLS
#LEFT_FORWARD_PIN - lf
LF_PIN = 13
LB_PIN = 19
RF_PIN = 18
RB_PIN = 12

pwm = []

GPIO.setup(LF_PIN, GPIO.OUT) # jobbra kanyarodáshoz
GPIO.setup(LB_PIN, GPIO.OUT)
GPIO.setup(RF_PIN, GPIO.OUT)
GPIO.setup(RB_PIN, GPIO.OUT)

pwm.append(GPIO.PWM(LF_PIN, 50))
pwm.append(GPIO.PWM(LB_PIN, 50))
pwm.append(GPIO.PWM(RF_PIN, 50))
pwm.append(GPIO.PWM(RB_PIN, 50))

def gyro():
    gyroskop_xout = read_word_2c(0x43)
    gyroskop_yout = read_word_2c(0x45)
    gyroskop_zout = read_word_2c(0x47)

    beschleunigung_xout = read_word_2c(0x3b)
    beschleunigung_yout = read_word_2c(0x3d)
    beschleunigung_zout = read_word_2c(0x3f)

    beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
    beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
    beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0

    '''
    gyroskop_xout+" "+gyroskop_yout+" "+gyroskop_zout+" "+
                       beschleunigung_xout+" "+beschleunigung_xout_skaliert +" "+
                       beschleunigung_yout+" "+beschleunigung_yout_skaliert+" "+
                       beschleunigung_zout+" "+beschleunigung_zout_skaliert+"\n"+
    

    saveSensorData("gyro.txt",
                   str(gyroskop_xout)+" "+str(gyroskop_yout)+" "+
                   str(gyroskop_zout) + " " +
                   str(get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert,
                                    beschleunigung_zout_skaliert))
                   + " " +
                   str(get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert,
                                    beschleunigung_zout_skaliert))
                   + "\n"
                   )

    '''
    print(    "X Rotation: ", get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert,
                                   beschleunigung_zout_skaliert)
    )
    print(    "Y Rotation: ", get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert,
                                   beschleunigung_zout_skaliert)
    )

def changePWM(pin,  goal):
    global pwm

    print("Pin: ", pin, "Freq: ", goal)

    pwm[pin].start(0)
    pwm[pin].ChangeDutyCycle(goal)


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
    #return render_template("index.html")
    #return render_template("joystick.html")
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
    # here we want to get the value of user (i.e. ?user=some-value)
    x = request.args.get('w')
    y = request.args.get('h')

    left=0
    right=0
    # akarom forgatni a tankot?
    # frekvenciátállítok
    if x<-10:
        # ballra akar menni
        left=abs(x)
        right=100-abs(x)
    elif x == 0:
        right=0
        left = 0
    elif x>10:
        right=abs(x)
        left = 100-abs(x)
    # hátra akar menni alar menni
    # ezzel váltom a pint
    if y<0:
        # Aktív
        p1=0
        p2=2
        # OFF
        p3 = 1
        p4 = 3
    elif y==0:
        changePWM(0,0)
        changePWM(1, 0)
        changePWM(2, 0)
        changePWM(3, 0)
    else:
        p1=1
        p2=3
        # OFF
        p3 = 0
        p4 = 2
    changePWM(p1,left)
    changePWM(p2, right)

    content = str(x)+"----"+str(y)
    mimetype="text/html"
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
    app.run(host=args["ip"], port=args["port"], debug=True,
            threaded=True, use_reloader=False)

# release the video stream pointer
vs.stop()

for p in pwm:
    p.stop()

GPIO.cleanup()
