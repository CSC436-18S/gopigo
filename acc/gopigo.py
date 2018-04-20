import sys
import time
import smbus


LEFT = 0                    #Used to denote the Left Motor
RIGHT = 1                   #Used to denote the Right Motor

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