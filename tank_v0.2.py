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

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print(
        "Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")
from http.server import BaseHTTPRequestHandler, HTTPServer
import time
import urllib.parse as urlparse
import smbus		#import SMBus module of I2C
import math
import threading
from multiprocessing import Pool
pool = Pool(processes=2)


# webszerver config

HOST_NAME = '0.0.0.0'
PORT_NUMBER = 9000

# USER controll enabled
# ha hamis, akkor a mozgatás le van tiltva.
UCE=True


# https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/ example alajpján
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

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


LEFT_FORWARD_PIN = 13
LEFT_BACKWARD_PIN = 19
RIGHT_FORWARD_PIN = 18
RIGHT_BACKWARD_PIN = 12

GPIO.setup(LEFT_FORWARD_PIN, GPIO.OUT)
GPIO.setup(LEFT_BACKWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_FORWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_BACKWARD_PIN, GPIO.OUT)

LEFT_FORWARD = GPIO.PWM(LEFT_FORWARD_PIN, 50)
LEFT_BACKWARD = GPIO.PWM(LEFT_BACKWARD_PIN, 50)
RIGHT_FORWARD = GPIO.PWM(RIGHT_FORWARD_PIN, 50)
RIGHT_BACKWARD = GPIO.PWM(RIGHT_BACKWARD_PIN, 50)  # channel=12 frequency=50Hz

LEFT_BACKWARD.start(0)
LEFT_BACKWARD_LAST = 0
LEFT_FORWARD.start(0)
LEFT_FORWARD_LAST = 0
RIGHT_FORWARD.start(0)
RIGHT_FORWARD_LAST = 0
RIGHT_BACKWARD.start(0)
RIGHT_BACKWARD_LAST = 0

def angle():
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
    return heading_angle

def changePWM(pin,last, goal):
    i = 1
    if last>goal:
        i = -i
    for dc in range(last, goal, i):
        pin.ChangeDutyCycle(dc)
        time.sleep(0.1)
        print("pwm",dc)

def run_process( arr):
    pin, last, goal = tuple(arr.split(","))
    changePWM(pin, last, goal)

def moveTank(forward_dir, side_dir):
    # értékellenörzés.
    if forward_dir > 100:
        forward_dir = 100

    if forward_dir < -100:
        forward_dir = -100

    if side_dir > 100:
        side_dir = 100

    if side_dir < -100:
        side_dir = -100
    global pool,LEFT_BACKWARD_LAST, LEFT_FORWARD, LEFT_FORWARD_LAST, RIGHT_FORWARD, RIGHT_FORWARD_LAST, RIGHT_BACKWARD, RIGHT_BACKWARD_LAST
    if abs(forward_dir) < 10 and side_dir > 20:
        print("balra")

        changePWM(LEFT_FORWARD, LEFT_FORWARD_LAST, 0)
        LEFT_FORWARD_LAST = 0
        changePWM(RIGHT_BACKWARD, RIGHT_BACKWARD_LAST, 0)
        RIGHT_BACKWARD_LAST = 0
        changePWM(LEFT_BACKWARD, LEFT_BACKWARD_LAST, abs(side_dir))
        LEFT_BACKWARD_LAST = 0
        changePWM(RIGHT_FORWARD, RIGHT_FORWARD_LAST, abs(side_dir))
        RIGHT_FORWARD_LAST = 0

    elif abs(forward_dir) < 10 and side_dir < -20:
        print("jobbra")

        RIGHT_FORWARD_LAST = 0
        process = ([LEFT_BACKWARD, LEFT_BACKWARD_LAST, 0],[RIGHT_FORWARD, RIGHT_FORWARD_LAST, 0])
        LEFT_BACKWARD_LAST = 0
        pool.map(changePWM, process)

        #changePWM(RIGHT_FORWARD, RIGHT_FORWARD_LAST, 0)
        print("start stage 2")
        #process=[(LEFT_FORWARD, LEFT_FORWARD_LAST, abs(side_dir)),(RIGHT_BACKWARD, RIGHT_BACKWARD_LAST, abs(side_dir))]
        #pool.map(run_process, process)

        #changePWM(LEFT_FORWARD, LEFT_FORWARD_LAST, abs(side_dir))
        LEFT_FORWARD_LAST = abs(side_dir)
        #changePWM(RIGHT_BACKWARD, RIGHT_BACKWARD_LAST, abs(side_dir))
        RIGHT_BACKWARD_LAST = abs(side_dir)


    else:
        if forward_dir > 0:
            print("elore")
        elif forward_dir < 0:
            print("hatra")
def start_server(path, port=9000):
    class MyHandler(BaseHTTPRequestHandler):
        def do_HEAD(self):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

        def do_GET(self):

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
                    moveTank(height, width)
                # van szög, van gyorsulás, ha minden igaz van minden adat az értélel letárolására, illetve visszatöltésére
                # TODO Visszatöltés megírása.
                heading_angle = angle()
                content = self.http_temp(True, "<p>Szög "+
                                         str(heading_angle)+
                                         '°</p>',False)
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


daemon = threading.Thread(name='daemon_server',
                          target=start_server,
                          args=('.', PORT_NUMBER))
daemon.setDaemon(True)  # Set as a daemon so it will be killed once the main thread is dead.
daemon.start()


try:
    time.sleep(100)
    # ez a rész függetlenül fut a webszerver hívásoktól

except KeyboardInterrupt:
    pass
LEFT_FORWARD.stop()
LEFT_BACKWARD.stop()
RIGHT_FORWARD.stop()
RIGHT_BACKWARD.stop()
GPIO.cleanup()
