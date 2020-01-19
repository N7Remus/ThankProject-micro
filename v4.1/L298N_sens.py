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

