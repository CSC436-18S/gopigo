import time
import smbus


LEFT = 0                    #Used to denote the Left Motor
RIGHT = 1                   #Used to denote the Right Motor

USS = 15                    #Pin used to access the Ultrasonic Sensor

ADDRESS = 0x08

ENC_READ_CMD = [53]
US_DIST = [117]

bus = smbus.SMBus(1)

def write_i2c_block(address, block):
    try:
        op = bus.write_i2c_block_data(address, 1, block)
        time.sleep(0.005)
        return op
    except IOError:
        return -1
    return 1

def enc_read(motor):
    write_i2c_block(ADDRESS, ENC_READ_CMD+[motor,0,0])
    #time.sleep(0.01)
    try:
        b1 = bus.read_byte(ADDRESS)
        b2 = bus.read_byte(ADDRESS)
    except IOError:
        return -1
    if b1 != -1 and b2 != -1:
        v = b1 * 256 + b2
        return v
    else:
        return None

def us_dist(pin):
    write_12c_block(ADDRESS, US_CMD+[pin,0,0])
    #time.sleep(0.08)
    try:
        b1 = bus.read_byte(ADDRESS)
        b2 = bus.read_byte(ADDRESS)
    except IOError:
        return None
    if b1 != -1 and b2 != -1:
        v = b1 * 256 + b2
        return v
    else:
        return None



def read_motor_speed():
    write_i2c_block(address,read_motor_speed_cmd+[unused,unused,unused])
    try:
        s1=bus.read_byte(address)
        s2=bus.read_byte(address)
    except IOError:
        return [-1,-1]
    return [s1,s2]


def set_left_speed(speed):
    """
	Sets the speed of the left motor. The speed should be in the range of
	[0, 255].
	Returns -1 if the motor speed change fails.
	:param int speed: The speed to set the left motor to. [0, 255]
	:return: A value indicating if the setting suceeded.
	:rtype: int
	"""
    if speed >255:
        speed =255
    elif speed <0:
        speed =0
    return write_i2c_block(address,set_left_speed_cmd+[speed,0,0])


def set_right_speed(speed):
    """
	Sets the speed of the right motor. The speed should be in the range of
	[0, 255].
	Returns -1 if the motor speed change fails.
	:param int speed: The speed to set the right motor to. [0, 255]
	:return: A value indicating if the setting suceeded.
	:rtype: int
	"""
    if speed >255:
        speed =255
    elif speed <0:
        speed =0
    return write_i2c_block(address,set_right_speed_cmd+[speed,0,0])
