"""
These functions have been modified from the gopigo library (GPLv3) in order
to remove some of the extra time.sleep times, so that the main loop can run
at a faster rate.

For the ACC, the running rate of the main loop can greatly impact the
performance as slower rates lead to slower response times.
"""

import time
import smbus


LEFT = 0                    # Used to denote the Left Motor
RIGHT = 1                   # Used to denote the Right Motor

USS = 15                    # Pin used to access the Ultrasonic Sensor

ADDRESS = 0x08

ENC_READ_CMD = [53]
US_DIST = [117]

bus = smbus.SMBus(1)

US_CMD = [117]
set_left_speed_cmd = [70]
set_right_speed_cmd = [71]
motor_fwd_cmd = [105]
trim_write_cmd = [31]
volt_cmd = [118]
stop_cmd = [120]
read_motor_speed_cmd = [114]

unused = 0

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

#Set speed of the both motors
#   arg:
#       speed-> 0-255
def set_speed(speed):
    """
    Sets the speed of the left and right motors to the given speed. The speed
    should be in the range of [0, 255].

    Sleeps for 0.1 seconds in between setting the left and right motor speeds.

    :param int speed: The speed to set the motors to. [0, 255]
    """
    if speed >255:
        speed =255
    elif speed <0:
        speed =0
    set_left_speed(speed)
    #time.sleep(.1)
    set_right_speed(speed)

#Set speed of the left motor
#   arg:
#       speed-> 0-255
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

#Set speed of the right motor
#   arg:
#       speed-> 0-255
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

#Write the trim value to EEPROM, where -100=0 and 100=200
def trim_write(value):
    if value>100:
        value=100
    elif value<-100:
        value=-100
    value+=100
    write_i2c_block(ADDRESS,trim_write_cmd+[value,0,0])

#Read voltage
#   return: voltage in V
def volt():
    write_i2c_block(ADDRESS,volt_cmd+[0,0,0])
    time.sleep(.1)
    try:
        b1=bus.read_byte(ADDRESS)
        b2=bus.read_byte(ADDRESS)
    except IOError:
        return -1

    if b1!=-1 and b2!=-1:
        v=b1*256+b2
        v=(5*float(v)/1024)/0.4
        return round(v,2)
    else:
        return -1

def enc_read(motor):
    write_i2c_block(ADDRESS, ENC_READ_CMD+[motor, 0, 0])
    #time.sleep(0.01)
    #time.sleep(0.08)
    try:
        b1 = bus.read_byte(ADDRESS)
        b2 = bus.read_byte(ADDRESS)
    except IOError:
        return -1
    if b1 != -1 and b2 != -1:
        v = b1 * 256 + b2
        return v
    else:
        return -1

#Stop the GoPiGo
def stop():
    """
    Brings the GoPiGo to a full stop.

    Returns -1 if the action fails.

    :return: A value indicating if the action suceeded.
    :rtype: int
    """
    status = write_i2c_block(ADDRESS,stop_cmd+[0,0,0])
    set_left_speed(0)
    set_right_speed(0)
    return status

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
    return write_i2c_block(ADDRESS,motor_fwd_cmd+[0,0,0])

def us_dist(pin):
    write_i2c_block(ADDRESS, US_CMD+[pin,0,0])
    time.sleep(0.01)
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
    write_i2c_block(ADDRESS,read_motor_speed_cmd+[unused,unused,unused])
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
