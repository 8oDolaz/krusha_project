import smbus2
from time import sleep


# Registers and their address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19  # Sample Rate Divider
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
SCALING_GYRO = 131
SCALING_ACC = 16384


class ImuSensor:
    def __init__(self, device_address):
        self.device_address = device_address
        smbus2.SMBus.write_byte_data(device_address, SMPLRT_DIV, 7)  # ?
        smbus2.SMBus.write_byte_data(device_address, PWR_MGMT_1, 1)  # H_RESET
        smbus2.SMBus.write_byte_data(device_address, CONFIG, 0)
        smbus2.SMBus.write_byte_data(device_address, GYRO_CONFIG, 0)
        smbus2.SMBus.write_byte_data(device_address, INT_ENABLE, 1)  # NOTHING? RESERVED

    def get_data(self, register):
        high = smbus2.SMBus.read_byte_data(self.device_address, register)
        low = smbus2.SMBus.read_byte_data(self.device_address, register + 1)
        # Concatenate higher and lower value
        value = ((high << 8) | low)
        # To get signed value
        if value >= 32768:
            value = value - 65536
        return value

    # Gyroscope data
    def get_gyro_data(self):
        gyro_x = self.get_data(GYRO_XOUT_H) / SCALING_GYRO
        gyro_y = self.get_data(GYRO_YOUT_H) / SCALING_GYRO
        gyro_z = self.get_data(GYRO_ZOUT_H) / SCALING_GYRO
        return gyro_x, gyro_y, gyro_z

    # Accelerometer data
    def get_acc_data(self):
        acc_x = self.get_data(ACCEL_XOUT_H) / SCALING_ACC
        acc_y = self.get_data(ACCEL_YOUT_H) / SCALING_ACC
        acc_z = self.get_data(ACCEL_ZOUT_H) / SCALING_ACC
        return acc_x, acc_y, acc_z
