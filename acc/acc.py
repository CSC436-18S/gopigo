from __future__ import print_function

import collections
import time
import traceback

import smbus
import gopigo

TRIM = 0#5#0
INITIAL_SPEED = 50
MAX_SPEED = 150 #200
MIN_SPEED = 30

INC_CONST = 100.0 #100.0

CRITICAL_DISTANCE = 10
SAFE_DISTANCE = 2 * CRITICAL_DISTANCE
ALERT_DISTANCE = 5 * SAFE_DISTANCE
SLOWDOWN_SPAN = (4.0/ 5.0) * (SAFE_DISTANCE - CRITICAL_DISTANCE)

SLOWING_DECCELLERATION = 50#100 # power units / second
SPEED_ACCELERATION = 40#100 # power units / second

STOP_THRESHOLD = 0.01

SAMPLE_SIZE = 10#20     # number of uss readings to sample for relative velocity
ALERT_THRESHOLD = 5.0 # 0.01

USS_ERROR = "USS_ERROR"
NOTHING_FOUND = "NOTHING_FOUND"

###############################################
# These functions have been modified from the gopigo library (GPLv3) in order
# to remove some of the extra time.sleep times, so that the main loop can run
# at a faster rate.
#
# For the ACC, the running rate of the main loop can greatly impact the
# performance as slower rates lead to slower response times.

ADDRESS = 0x08
ENC_READ_CMD = [53]

LEFT = 0
RIGHT = 1

USS = 15

BUS = smbus.SMBus(1)

def write_i2c_block(address, block):
    try:
        op = BUS.write_i2c_block_data(address, 1, block)
        time.sleep(0.005)
        return op
    except IOError:
        return -1
    return 1

def enc_read(motor):
    write_i2c_block(ADDRESS, ENC_READ_CMD+[motor, 0, 0])
    #time.sleep(0.01)
    #time.sleep(0.08)
    try:
        b1 = BUS.read_byte(ADDRESS)
        b2 = BUS.read_byte(ADDRESS)
    except IOError:
        return -1
    if b1 != -1 and b2 != -1:
        v = b1 * 256 + b2
        return v
    else:
        return -1

US_CMD = [117]

def us_dist(pin):
    write_i2c_block(ADDRESS, US_CMD+[pin, 0, 0])
    time.sleep(0.01)
    #time.sleep(0.08)
    try:
        b1 = BUS.read_byte(ADDRESS)
        b2 = BUS.read_byte(ADDRESS)
    except IOError:
        return -1
    if b1 != -1 and b2 != -1:
        v = b1 * 256 + b2
        return v
    else:
        return -1

###############################################

def get_inc(speed):
    """
    Returns the power amount to use in straightness correcting.

    It is based on the current speed, so that at higher speeds it corrects with
    less, and at lower speeds it corrects with more.

    :param float speed: The current speed that the rover is going (power units)
    :return: The correction power change (power units)
    :rtype: float
    """
    if speed < 0.1 and speed > -0.1:
        return 0
    else:
        return (9.0 / (speed / INC_CONST)) / 1.5

def get_deccelleration(speed):
    """
    Returns the deccelleration amount to use when slowing down when within the
    safe distance.

    It is based on the current speed so that at higher speeds it decelerates
    more, and at lower speeds it deccellerates less.

    :param float speed: The current speed that the rover is going (power units)
    :return: The deccelleration to apply to the rover's speed (power units / seconds)
    :rtype: float
    """
    return (speed ** 2.0) / (2.0 * SLOWDOWN_SPAN)

def main(command_queue):
#def main():
    speed = INITIAL_SPEED

    gopigo.trim_write(TRIM)

    time.sleep(0.1)
    print("Volt: " + str(gopigo.volt()))

    time.sleep(0.1)
    initial_ticks_left = enc_read(LEFT)
    time.sleep(0.1)
    initial_ticks_right = enc_read(RIGHT)

    print("Initial\tL: " + str(initial_ticks_left) + "\tR: " + str(initial_ticks_right))

    print("Critical: " + str(CRITICAL_DISTANCE))
    print("Safe:     " + str(SAFE_DISTANCE))
    print("Alert:    " + str(ALERT_DISTANCE))

    dists = collections.deque(maxlen=SAMPLE_SIZE)
    dts = collections.deque(maxlen=SAMPLE_SIZE)

    elapsed_ticks_left = 0
    elapsed_ticks_right = 0

    try:
        gopigo.set_speed(0)
        gopigo.fwd()

        t = time.time()
        while True:
            if speed < 0:
                speed = 0

            print("========================")
            #if not command_queue.empty():
            #    comm = command_queue.get()
            #    global MAX_SPEED
            #    global SAFE_DISTANCE
            #    MAX_SPEED = comm[0]
            #    SAFE_DISTANCE = comm[1]

            #    print(comm)

            dt = time.time() - t
            t = time.time()

            #time.sleep(0.1)

            print("Time: " + str(dt))

            dist = get_dist()
            print("Dist: " + str(dist))

            if not isinstance(dist, str):
                dists.append(float(dist))
                dts.append(float(dt))

            rel_speed = None
            if len(dists) > 9:
                rel_speed = calculate_relative_speed(dists, dts)
                print("Rel speed: " + str(rel_speed))

            if (isinstance(dist, str) and dist != NOTHING_FOUND) or dist < CRITICAL_DISTANCE:
                print("< Critical")
                stop_until_safe_distance()
                speed = 0
                t = time.time()
            elif dist < SAFE_DISTANCE:
                print("< Safe")
                if speed > STOP_THRESHOLD:
                    #speed = speed - dt * SPEED_DECCELLERATION
                    speed = speed - dt * get_deccelleration(speed)
                else:
                    speed = 0
            elif speed > MAX_SPEED:
                print("Slowing down")
                speed = speed - dt * SLOWING_DECCELLERATION
            elif dist < ALERT_DISTANCE and rel_speed is not None:
                speed = handle_alert_distance(speed, rel_speed, dt)
            elif speed < MAX_SPEED:
                print("Speeding up")
                speed = speed + dt * SPEED_ACCELERATION
                #speed = speed - dt * get_deccelleration(speed)

            elapsed_ticks_left, elapsed_ticks_right = \
                read_enc_ticks(initial_ticks_left, initial_ticks_right)

            print("L: " + str(elapsed_ticks_left) + "\tR: " + str(elapsed_ticks_right))

            l_diff, r_diff = straightness_correction(speed, elapsed_ticks_left, elapsed_ticks_right)

            if elapsed_ticks_left >= 0 and elapsed_ticks_right >= 0:
                set_speed_lr(speed, l_diff, r_diff)
            else:
                set_speed_lr(speed, 0, 0)

            print("Speed: " + str(speed))

    except (KeyboardInterrupt, Exception):
        traceback.print_exc()
        gopigo.stop()
    gopigo.stop()

def handle_alert_distance(speed, rel_speed, dt):
    """
    Determines the new speed of the rover when it is in the alert distance
    in order to attempt to match the speed of the obstacle.

    Keeps going at a minimum speed even if the obstacle is not moving so that
    it will stop around the safe distance.

    :param float speed: The current speed of the rover (power units)
    :param float rel_speed: The speed of the obstacle relative to the rover (cm / s)
    :param float dt: The change in time for the previous run of the main loop (s)
    :return: The new speed of the rover (power units)
    :rtype: float
    """
    if rel_speed > ALERT_THRESHOLD:
        print("Alert speeding")
        new_speed = speed + dt * SPEED_ACCELERATION

        return new_speed
    elif rel_speed < -ALERT_THRESHOLD:
        print("Alert slowing")
        new_speed = speed - dt * SLOWING_DECCELLERATION

        if new_speed < MIN_SPEED:
            new_speed = MIN_SPEED

        return new_speed
    else:
        print("Alert stable")
        return speed

def straightness_correction(speed, elapsed_ticks_left, elapsed_ticks_right):
    """
    Returns the power adjustments to make to each motor to correct the
    straightness of the path of the rover.

    :param float speed: The speed that the rover is going at (power units)
    :param int elapsed_ticks_left: The number of left ticks (ticks)
    :param int elapsed_ticks_right: The number of right ticks (ticks)
    :return: The left and right motor speed adjustments (power units)
    :rtype: tuple[float, float]
    """
    if elapsed_ticks_left > elapsed_ticks_right:
        print("Right slow")
        return (-get_inc(speed), get_inc(speed))
    elif elapsed_ticks_left < elapsed_ticks_right:
        print("Left slow")
        return (get_inc(speed), -get_inc(speed))
    else:
        print("Equal")
        return (0, 0)

def read_enc_ticks(initial_ticks_left, initial_ticks_right):
    time.sleep(0.01)
    elapsed_ticks_left = enc_read(LEFT) - initial_ticks_left
    #time.sleep(0.005)
    time.sleep(0.01)
    elapsed_ticks_right = enc_read(RIGHT) - initial_ticks_right
    #time.sleep(0.005)

    return (elapsed_ticks_left, elapsed_ticks_right)

def set_speed_lr(speed, l_diff, r_diff):
    if speed >= MIN_SPEED:
        gopigo.set_left_speed(int(speed + l_diff))
        gopigo.set_right_speed(int(speed + r_diff))
    else:
        gopigo.set_left_speed(0)
        gopigo.set_right_speed(0)

def calculate_relative_speed(dists, dts):
    old_dist = sum(list(dists)[0:len(dists) / 2]) / (len(dists) / 2)
    new_dist = sum(list(dists)[len(dists) / 2:]) / (len(dists) / 2)

    old_dt = sum(list(dts)[0:len(dts) / 2])
    new_dt = sum(list(dts)[len(dts) / 2:])

    rel_speed = (new_dist - old_dist) / ((new_dt + old_dt) / 2.0)

    return rel_speed

def get_dist():
    time.sleep(0.01)
    dist = us_dist(USS)

    if dist == -1:
        return USS_ERROR
    elif dist == 0 or dist == 1:
        return NOTHING_FOUND
    else:
        return dist

def stop_until_safe_distance():
    gopigo.stop()
    dist = get_dist()
    while (isinstance(dist, str) and dist != NOTHING_FOUND) or dist < SAFE_DISTANCE:
        dist = get_dist()

    gopigo.set_speed(0)
    gopigo.fwd()

if __name__ == "__main__":
    main()
