import time
import smbus


LEFT = 0                    #Used to denote the Left Motor
RIGHT = 1                   #Used to denote the Right Motor

USS = 15                    #Pin used to access the Ultrasonic Sensor

ADDRESS = 0x08

ENC_READ_CMD = [53]
US_DIST = [117]

bus = smbus.SMBus(1)

US_CMD = [117]
set_left_speed_cmd = [70]
set_right_speed_cmd = [71]

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

def fwd(dist=0): #distance is in cm
    """
    Starts moving the GoPiGo forward at the currently set motor speeds.
    Takes an optional parameter to indicate a specific distance to move
    forward. If not given, negative, or zero then it will move forward until
    another direction or stop function is called.
    Returns -1 if the action fails.
    :param int dist: The distance in cm to move forward.
    :return: A value indicating if the action suceeded.
    :rtype: int
    """
    try:
        if dist>0:
            # this casting to int doesn't seem necessary
            pulse=int(PPR*(dist//WHEEL_CIRC) )
            enc_tgt(1,1,pulse)
    except Exception as e:
        print ("gopigo fwd: {}".format(e))
        pass
    return write_i2c_block(address,motor_fwd_cmd+[0,0,0])

def us_dist(pin):
    write_i2c_block(ADDRESS, US_CMD+[pin,0,0])
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
        s1=bus.read_byte(ADDRESS)
        s2=bus.read_byte(ADDRESS)
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
    return write_i2c_block(ADDRESS,set_left_speed_cmd+[speed,0,0])


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
    return write_i2c_block(ADDRESS,set_right_speed_cmd+[speed,0,0])
