# Raspberry specifikus modulok
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print(
        "Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")
import time


# https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/ example alajpján
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(13, GPIO.OUT)

p = GPIO.PWM(13, 50)  # channel=12 frequency=50Hz
p.start(0)
try:
    while 1:
        for dc in range(0, 101, 5):
            print(dc)
            p.ChangeDutyCycle(dc)
            time.sleep(0.1)
        for dc in range(100, -1, -5):
            print(dc)
            p.ChangeDutyCycle(dc)
            time.sleep(0.1)
except KeyboardInterrupt:
    pass
p.stop()
GPIO.cleanup()