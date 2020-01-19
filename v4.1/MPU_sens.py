# Magnetometer

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


class MPU_class:

    def __init__(self, Device_Address_MPU=0x1e # HMC5883L magnetometer device address
                 ):
        self.Device_Address = Device_Address_MPU
        self.angle = 0
    def Magnetometer_Init(self):
        bus.write_byte_data(self.Device_Address, 0x37, 0x02)
        bus.write_byte_data(self.Device_Address, 0x6A, 0x00)
        bus.write_byte_data(self.Device_Address, 0x6B, 0x00)

        '''# write to Configuration Register A
        bus.write_byte_data(self.Device_Address, Register_A, 0x70)

        # Write to Configuration Register B for gain
        bus.write_byte_data(self.Device_Address, Register_B, 0xa0)

        # Write to mode Register for selecting mode
        bus.write_byte_data(self.Device_Address, Register_mode, 0)
        '''
    def read_raw_data(self,addr):
        # Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(self.Device_Address, addr)
        low = bus.read_byte_data(self.Device_Address, addr + 1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # to get signed value from mpu6050
        if value > 32768:
            value = value - 65536
        return value

    def printPin(self):
        print("Az iránytű modul modul: " + str(self.Device_Address) + " címen kommunikál")

    def update(self):

    def htmlFormat(self):
        content = "Távolság : " + str(self.angle)
        return content
