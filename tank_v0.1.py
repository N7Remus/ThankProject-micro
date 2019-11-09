'''
 +-----+-----+---------+------+---+---Pi 3B--+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
 |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
 |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 0 | IN   | TxD     | 15  | 14  |
 |     |     |      0v |      |   |  9 || 10 | 1 | IN   | RxD     | 16  | 15  |
 |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
 |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
 |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
 |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
 |  10 |  12 |    MOSI |   IN | 0 | 19 || 20 |   |      | 0v      |     |     |
 |   9 |  13 |    MISO |   IN | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
 |  11 |  14 |    SCLK |   IN | 0 | 23 || 24 | 1 | IN   | CE0     | 10  | 8   |
 |     |     |      0v |      |   | 25 || 26 | 1 | IN   | CE1     | 11  | 7   |
 |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
 |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
 |   6 |  22 | GPIO.22 |  OUT | 0 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
 |  13 |  23 | GPIO.23 | ALT0 | 0 | 33 || 34 |   |      | 0v      |     |     |
 |  19 |  24 | GPIO.24 |  OUT | 1 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
 |  26 |  25 | GPIO.25 |  OUT | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
 |     |     |      0v |      |   | 39 || 40 | 0 | OUT  | GPIO.29 | 29  | 21  |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+---Pi 3B--+---+------+---------+-----+-----+
verzió mic2-py_v2_1
'''
from http.server import BaseHTTPRequestHandler, HTTPServer
import urllib.parse as urlparse
import smbus		#import SMBus module of I2C
from time import sleep  #import sleep
import math

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print(
        "Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")
import time, os, sys
import threading

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


bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x1e  # HMC5883L magnetometer device address

Magnetometer_Init()  # initialize HMC5883L magnetometer

GPIO.setmode(GPIO.BCM) # Bcm kiosztás alapján címzem a pineket.
GPIO.setwarnings(False)
# set GPIO Pins
GPIO_TRIGGER = 14
GPIO_ECHO = 15

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# pwm pinek 13,19 | 18,12
LEFT_FORWARD = 13
LEFT_BACKWARD = 19
RIGHT_FORWARD = 18
RIGHT_BACKWARD = 12

# RGB let pinek.
RED = 17
GREEN = 27
BLUE = 22
# Feéhasználó adhat parancsot?
UCE = True

# Left
GPIO.setup(LEFT_FORWARD, GPIO.OUT)
GPIO.setup(LEFT_BACKWARD, GPIO.OUT)
# right
GPIO.setup(RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(RIGHT_BACKWARD, GPIO.OUT)

GPIO.setup(RED, GPIO.OUT)
GPIO.setup(GREEN, GPIO.OUT)
GPIO.setup(BLUE, GPIO.OUT)

pwm = {}

pwm[LEFT_FORWARD] = GPIO.PWM(LEFT_FORWARD, 1000)
pwm[LEFT_BACKWARD] = GPIO.PWM(LEFT_BACKWARD, 1000)
pwm[RIGHT_FORWARD] = GPIO.PWM(RIGHT_FORWARD, 1000)
pwm[RIGHT_BACKWARD] = GPIO.PWM(RIGHT_BACKWARD, 1000)

HOST_NAME = '0.0.0.0'
PORT_NUMBER = 9000

dist = 0
dist_color = "red"
width = 0
height = 0


def analogWrite(pin, freq):
    global pwm

    print("Pin: ", pin, "Freq: ", freq)

    pwm[pin].start(0)
    pwm[pin].ChangeDutyCycle(freq)


def mover(forward_dir, side_dir):
    # értékellenörzés.
    if forward_dir > 100:
        forward_dir = 100

    if forward_dir < -100:
        forward_dir = -100

    if side_dir > 100:
        side_dir = 100

    if side_dir < -100:
        side_dir = -100

    tmp = abs(forward_dir) - abs(side_dir)
    global pwm

    if abs(forward_dir) <= 10:
        for i in pwm:
            pwm[i].ChangeDutyCycle(0)
        GPIO.output(LEFT_FORWARD, False)
        GPIO.output(LEFT_BACKWARD, False)
        GPIO.output(RIGHT_FORWARD, False)
        GPIO.output(RIGHT_BACKWARD, False)

    else:
        if forward_dir > 0:

            pwm[LEFT_BACKWARD].stop()
            pwm[RIGHT_BACKWARD].stop()
            GPIO.output(LEFT_BACKWARD, False)
            GPIO.output(RIGHT_BACKWARD, False)

            if (side_dir > 0):
                analogWrite(LEFT_FORWARD, round((tmp if tmp > 0 else 0)))
                analogWrite(RIGHT_FORWARD, round((abs(forward_dir))))

            if (side_dir < 0):
                analogWrite(LEFT_FORWARD, round((abs(forward_dir))))
                analogWrite(RIGHT_FORWARD, round((tmp if tmp > 0 else 0)))

        if (forward_dir < 0):

            pwm[LEFT_FORWARD].stop()
            pwm[RIGHT_FORWARD].stop()
            GPIO.output(LEFT_FORWARD, False)
            GPIO.output(RIGHT_FORWARD, False)

            if side_dir > 0:
                analogWrite(LEFT_BACKWARD, round((tmp if tmp > 0 else 0)))
                analogWrite(RIGHT_BACKWARD, round((abs(forward_dir))))

            if side_dir < 0:
                analogWrite(LEFT_BACKWARD, round((abs(forward_dir))))
                analogWrite(RIGHT_BACKWARD, round((tmp if tmp > 0 else 0)))


def start_server(path, port=9000):
    class MyHandler(BaseHTTPRequestHandler):
        def do_HEAD(self):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

        def do_GET(self):
            paths = {
                '/foo': {'status': 200},
                '/ajax': {'status': 200},
                '/bar': {'status': 302},
                '/baz': {'status': 404},
                '/qux': {'status': 500}
            }

            if self.path in paths:
                self.respond(paths[self.path])
            else:
                self.respond({'status': 200})

        #            self.respond({'status': 500})

        def http_temp(self, ajax, file="",isfile=True):
            s = ""
            if file=="":
                with open("html_page/basic.html", "r") as f:
                    s = f.read()
            else:

                if isfile:
                    with open(file, "r") as f:
                        s = f.read()
                else:
                    s = file
            return str(s)

        def handle_http(self, status_code, path):
            self.send_response(status_code)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            print("PATH:"+path)
            if path.startswith("/ajax"):
                parsed = urlparse.urlparse(path)
                try:
                    # print(urlparse.parse_qs(parsed.query)["w"],urlparse.parse_qs(parsed.query)["h"])
                    global width, height

                    width = int(float(urlparse.parse_qs(parsed.query)["w"][0]))
                    height = int(float(urlparse.parse_qs(parsed.query)["h"][0]))

                    print("Width: ", width, "Heigth: ", height)
                except Exception as e:
                    print(e)
                if UCE:
                    mover(height, width)
                # távolságmérő szenzor infója
                global dist
                # magnetométer infója
                # Read Accelerometer raw value
                x = read_raw_data(X_axis_H)
                z = read_raw_data(Z_axis_H)
                y = read_raw_data(Y_axis_H)

                print(x, y, z)

                heading = math.atan2(y, x) + declination

                # Due to declination check for >360 degree
                if (heading > 2 * pi):
                    heading = heading - 2 * pi

                # check for sign
                if (heading < 0):
                    heading = heading + 2 * pi

                # convert into angle
                heading_angle = int(heading * 180 / pi)
                # van szög, van gyorsulás, ha minden igaz van minden adat az értélel letárolására, illetve visszatöltésére
                # TODO Visszatöltés megírása.

                content = self.http_temp(True, "<p>Szög "+
                                         str(heading_angle)+
                                         '°</p><p style="color:'+
                                         dist_color+'">'+
                                         str(int(dist))+
                                         'cm</p>',False)
                # content = self.http_temp(True, path)
            elif path=="/":
                content = self.http_temp(False)
            else:
                print("html_page"+path)
                content = self.http_temp(False, file="html_page"+path)


            return bytes(content, 'UTF-8')

        def respond(self, opts):
            response = self.handle_http(opts['status'], self.path)
            self.wfile.write(response)

    if __name__ == '__main__':
        server_class = HTTPServer
        httpd = server_class((HOST_NAME, PORT_NUMBER), MyHandler)
        print(time.asctime(), 'Server Starts - %s:%s' % (HOST_NAME, PORT_NUMBER))
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        httpd.server_close()
        print(time.asctime(), 'Server Stops - %s:%s' % (HOST_NAME, PORT_NUMBER))


# Start the server in a new thread
port = 9000
daemon = threading.Thread(name='daemon_server',
                          target=start_server,
                          args=('.', port))
daemon.setDaemon(True)  # Set as a daemon so it will be killed once the main thread is dead.
daemon.start()


# Open the web browser
# webbrowser.open('http://localhost:{}'.format(port))
def distance():
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

    return distance



try:
    while True:
        dist = distance()
        if dist < 30:
            UCE = False
            dist_color="red"
            GPIO.output(RED, True)
            GPIO.output(BLUE, False)
            GPIO.output(GREEN, False)
            analogWrite(LEFT_BACKWARD, 50)
            time.sleep(1.75)
            analogWrite(LEFT_BACKWARD, 0)

            UCE = True

        elif dist < 130:
            dist_color="blue"
            GPIO.output(RED, False)
            GPIO.output(BLUE, True)
            GPIO.output(GREEN, False)
        elif dist > 400:
            dist_color = "greem"
            GPIO.output(RED, True)
            GPIO.output(BLUE, True)
            GPIO.output(GREEN, True)
        else:
            dist_color = "white"
            GPIO.output(RED, False)
            GPIO.output(BLUE, False)
            GPIO.output(GREEN, True)
        print("Measured Distance = %.1f cm" % dist)
        time.sleep(1)
# Reset by pressing CTRL + C
except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
