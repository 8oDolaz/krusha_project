import smbus2


# Registers and their address
PWR_MGMT_1   = 0x6B   # Power Managment, 107 
SMPLRT_DIV   = 0x19   # Sample Rate Divider, 25
CONFIG       = 0x1A   # Configuration, 26
GYRO_CONFIG  = 0x1B   # Gyro Configuration, 27
INT_ENABLE   = 0x38   # Interrupt enable, 56
ACCEL_XOUT_H = 0x3B   # Accelerometer on X axis output, 59
ACCEL_YOUT_H = 0x3D   # Accelerometer on Y axis output, 61
ACCEL_ZOUT_H = 0x3F   # Accelerometer on Z axis output, 63
GYRO_XOUT_H  = 0x43   # Gyroscope on X axis output, 67
GYRO_YOUT_H  = 0x45   # Gyroscope on Y axis output, 69
GYRO_ZOUT_H  = 0x47   # Gyroscope on Z axis output, 71

# Measurements constanst
SCALING_GYRO = 131    # Scailing coefficient for Gyroscope data
SCALING_ACC = 16384   # Scailing coefficient for Accelerometer data


class IMU_Sensor:
    def __init__(self, device_address):
        self.device_address = device_address

        smbus2.SMBus.write_byte_data(device_address, SMPLRT_DIV, 7)
        smbus2.SMBus.write_byte_data(device_address, PWR_MGMT_1, 1)
        smbus2.SMBus.write_byte_data(device_address, CONFIG, 0)
        smbus2.SMBus.write_byte_data(device_address, GYRO_CONFIG, 0)
        smbus2.SMBus.write_byte_data(device_address, INT_ENABLE, 1)


    @staticmethod
    def __concatinate_registers(high, low, register_size=8):
        # Concatenate value from higher and lower register of register_size bit to get one value
        #
        # Input parameters:
        #   high: High register
        #   low: Low register
        #
        # Return integer value of combined registers

        return (high << register_size) | low
    

    @staticmethod
    def __signed_value(unsighned_value):
        # Converts an unsigned 16 bit value to a signed value.
        # 
        # Input parameters:
        #   unsighned_value: The unsigned 16 bit value to convert.
        #
        # Returns the signed value.

        if unsighned_value > 2 ** 15:
            return unsighned_value - (2 ** 16)
        return unsighned_value


    def __get_data(self, register):
        # Reads data from a specified register.
        #
        # Input parametrs:
        #   register (int): The register address to read from.
        #
        # Return integer value read from the register

        high = smbus2.SMBus.read_byte_data(self.device_address, register)
        low = smbus2.SMBus.read_byte_data(self.device_address, register + 1)
  
        value = self.__concatinate_registers(high, low)
        value = self.__signed_value(value)

        return value


    def get_gyro_data(self):
        # Reads the gyroscope data from the sensor.
        # 
        # Returns a tuple containing the gyroscope data in the x, y, and z axes.

        gyro_x = self.__get_data(GYRO_XOUT_H) / SCALING_GYRO
        gyro_y = self.__get_data(GYRO_YOUT_H) / SCALING_GYRO
        gyro_z = self.__get_data(GYRO_ZOUT_H) / SCALING_GYRO

        return gyro_x, gyro_y, gyro_z


    def get_acc_data(self):
        # Reads the accelerometer data from the sensor.
        # 
        # Returns a tuple containing the accelerometer data in the x, y, and z axes.

        acc_x = self.__get_data(ACCEL_XOUT_H) / SCALING_ACC
        acc_y = self.__get_data(ACCEL_YOUT_H) / SCALING_ACC
        acc_z = self.__get_data(ACCEL_ZOUT_H) / SCALING_ACC

        return acc_x, acc_y, acc_z
