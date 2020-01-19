class HCSR_class:

    # https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/ alapján
    def __init__(self, trigger=23, echo=24):
        self.trigger = trigger
        self.echo = echo
        self.distance = 0
        GPIO.setup(trigger, GPIO.OUT)
        GPIO.setup(trigger, GPIO.OUT)

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
