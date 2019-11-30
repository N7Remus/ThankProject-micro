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
import smbus  # import SMBus module of I2C
import math
import threading

# webszerver config

HOST_NAME = '0.0.0.0'
PORT_NUMBER = 9000

# USER controll enabled
# ha hamis, akkor a mozgatás le van tiltva.
UCE = True

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

LEFT_FORWARD_PIN = 13
LEFT_BACKWARD_PIN = 19
RIGHT_FORWARD_PIN = 18
RIGHT_BACKWARD_PIN = 12

GPIO.setup(LEFT_FORWARD_PIN, GPIO.OUT)
GPIO.setup(LEFT_BACKWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_FORWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_BACKWARD_PIN, GPIO.OUT)
pwm = []
LEFT_FORWARD = 0
pwm.append(GPIO.PWM(LEFT_FORWARD_PIN, 50))
LEFT_BACKWARD = 1
pwm.append(GPIO.PWM(LEFT_BACKWARD_PIN, 50))
RIGHT_FORWARD = 2
pwm.append(GPIO.PWM(RIGHT_FORWARD_PIN, 50))
RIGHT_BACKWARD = 3
pwm.append(GPIO.PWM(RIGHT_BACKWARD_PIN, 50))  # channel=12 frequency=50Hz

for p in pwm:
    p.start(0)

LEFT_BACKWARD_LAST = 0
LEFT_FORWARD_LAST = 0
RIGHT_FORWARD_LAST = 0
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

    print(    "X Rotation: ", get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert,
                                   beschleunigung_zout_skaliert)
    )
    print(    "Y Rotation: ", get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert,
                                   beschleunigung_zout_skaliert)
    )


def changePWM(pin, last, goal):
    global pwm

    print("Pin: ", pin, "Freq: ", goal)

    pwm[pin].start(0)
    pwm[pin].ChangeDutyCycle(goal)

def saveSensorData(FILE,DATA):
    #CSV vagy SQL - megolással lehetne megoldani,még nem biztos hogy mivel csináljuk ezért van függvényben.
    with open(FILE, "a") as myFile:
        myFile.write(DATA)


def moveTank(forward_dir, side_dir):
    # értékellenörzés.
    if forward_dir > 100:
        forward_dir = 100
    elif forward_dir < -100:
        forward_dir = -100

    if side_dir > 100:
        side_dir = 100
    elif side_dir < -100:
        side_dir = -100

    side_dir = side_dir // 2
    forward_dir = forward_dir // 2

    global LEFT_BACKWARD_LAST, LEFT_FORWARD, LEFT_FORWARD_LAST, RIGHT_FORWARD, RIGHT_FORWARD_LAST, RIGHT_BACKWARD, RIGHT_BACKWARD_LAST
    # fordulás
    if abs(forward_dir) < 10 and side_dir > 15:
        print("balra")

        changePWM(LEFT_BACKWARD, LEFT_BACKWARD_LAST, 0)
        changePWM(RIGHT_FORWARD, RIGHT_FORWARD_LAST, 0)
        LEFT_BACKWARD_LAST = 0
        RIGHT_FORWARD_LAST = 0

        changePWM(LEFT_FORWARD, LEFT_FORWARD_LAST, abs(side_dir))
        changePWM(RIGHT_BACKWARD, RIGHT_BACKWARD_LAST, abs(side_dir))
        LEFT_FORWARD_LAST = abs(side_dir)
        RIGHT_BACKWARD_LAST = abs(side_dir)
    elif abs(forward_dir) < 10 and abs(side_dir) > 15:
        print("jobbra")

        changePWM(LEFT_FORWARD, LEFT_FORWARD_LAST, 0)
        changePWM(RIGHT_BACKWARD, RIGHT_BACKWARD_LAST, 0)
        LEFT_FORWARD_LAST = 0
        RIGHT_BACKWARD_LAST = 0

        changePWM(LEFT_BACKWARD, LEFT_BACKWARD_LAST, abs(side_dir))
        changePWM(RIGHT_FORWARD, RIGHT_FORWARD_LAST, abs(side_dir))
        LEFT_BACKWARD_LAST = abs(side_dir)
        RIGHT_FORWARD_LAST = abs(side_dir)
    else:

        jobbra = abs(side_dir) if side_dir > 0 else 0
        ballra = abs(side_dir) if side_dir < 0 else 0

        if forward_dir > 0:
            print("elore")

            changePWM(LEFT_BACKWARD, LEFT_BACKWARD_LAST, 0)
            changePWM(RIGHT_BACKWARD, RIGHT_BACKWARD_LAST, 0)

            LEFT_BACKWARD_LAST = 0
            RIGHT_BACKWARD_LAST = 0

            changePWM(RIGHT_FORWARD, RIGHT_FORWARD_LAST, abs(forward_dir) + ballra)
            changePWM(LEFT_FORWARD, LEFT_FORWARD_LAST, abs(forward_dir) + jobbra)

            RIGHT_FORWARD_LAST = abs(forward_dir) + ballra
            LEFT_FORWARD_LAST = abs(forward_dir) + jobbra

        elif forward_dir < 0:
            print("hatra")
            changePWM(RIGHT_FORWARD, RIGHT_FORWARD_LAST, 0)
            changePWM(LEFT_FORWARD, LEFT_FORWARD_LAST, 0)

            RIGHT_FORWARD_LAST = 0
            LEFT_FORWARD_LAST = 0

            changePWM(RIGHT_BACKWARD, RIGHT_BACKWARD_LAST, abs(forward_dir) + ballra)
            changePWM(LEFT_BACKWARD, LEFT_BACKWARD_LAST, abs(forward_dir) + jobbra)

            RIGHT_BACKWARD_LAST = abs(forward_dir) + ballra
            LEFT_BACKWARD_LAST = abs(forward_dir) + jobbra
        else:

            changePWM(LEFT_BACKWARD, LEFT_BACKWARD_LAST, 0)
            changePWM(RIGHT_BACKWARD, RIGHT_BACKWARD_LAST, 0)

            LEFT_BACKWARD_LAST = 0
            RIGHT_BACKWARD_LAST = 0

            changePWM(RIGHT_FORWARD, RIGHT_FORWARD_LAST, 0)
            changePWM(LEFT_FORWARD, LEFT_FORWARD_LAST, 0)

            RIGHT_FORWARD_LAST = 0
            LEFT_FORWARD_LAST = 0


def start_server(path, port=9000):
    class MyHandler(BaseHTTPRequestHandler):
        def do_HEAD(self):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

        def do_GET(self):
            self.respond({'status': 200})

        def http_temp(self, ajax, file="", isfile=True):
            s = ""
            if file == "":
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
            print("PATH:" + path)
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

                saveSensorData("asd",heading_angle+" "+width+" "+height)

                content = self.http_temp(True, "<p>Szög " +
                                         str(heading_angle) +
                                         '°</p>', False)
            elif path == "/":
                content = self.http_temp(False)
            else:
                print("html_page" + path)
                content = self.http_temp(False, file="html_page" + path)

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
for p in pwm:
    p.stop()

GPIO.cleanup()
