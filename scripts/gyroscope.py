import smbus		#import SMBus module of I2C
from time import sleep  #import sleep
import math

# some MPU6050 Registers and their Address
Register_A     = 0              # Address of Configuration register A
Register_B     = 0x01           # Address of configuration register B
Register_mode  = 0x02           # Address of mode register

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x1e   # HMC5883L magnetometer device address

X_axis_H    = 0x03              # Address of X-axis MSB data register
Z_axis_H    = 0x05              # Address of Z-axis MSB data register
Y_axis_H    = 0x07              # Address of Y-axis MSB data register

class Gyroscope:

    scale = 0.92
    declination = -0.00669          # Define declination angle of location where measurement going to be done
    x_offset = -163
    y_offset = -78

    def __init__(self):
        # Write to Configuration Register A
        bus.write_byte_data(Device_Address, Register_A, 0b01110000)

        # Write to Configuration Register B for gain
        bus.write_byte_data(Device_Address, Register_B, 0b00100000)

        # Write to mode Register for selecting mode
        bus.write_byte_data(Device_Address, Register_mode, 0)

    def read_raw_data(self, addr):
        # Read raw 16-bit value
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        # Concatenate higher and lower value
        value = ((high << 8) | low)

        # To get signed value from module
        if(value > 32768):
            value = value - 65536
        return value

    def get_reading(self):
        # Read Accelerometer raw value
        x = (self.read_raw_data(X_axis_H)-self.x_offset) * self.scale
        z = self.read_raw_data(Z_axis_H) * self.scale
        y = (self.read_raw_data(Y_axis_H)-self.y_offset) * self.scale

        heading = math.atan2(y, x) + self.declination

        # Due to declination check for > 360 degrees
        if(heading > 2*math.pi):
            heading = heading - 2*math.pi

        # Check for sign
        if(heading < 0):
            heading = heading + 2*math.pi

        # Convert into angle
        heading_angle = int(heading * 180/math.pi)

        return heading_angle
