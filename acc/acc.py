from __future__ import print_function

import collections
import time
import traceback

import gopigo

import commands

TRIM = 0#5#0
INITIAL_SPEED = 50
MAX_SPEED = 150 #200
MIN_SPEED = 30

INC_CONST = 100.0 #100.0

CRITICAL_DISTANCE = 10
SAFE_DISTANCE = 2 * CRITICAL_DISTANCE
#ALERT_DISTANCE = 5 * SAFE_DISTANCE
ALERT_DISTANCE_CONST = 3
SLOWDOWN_SPAN = (4.0/ 5.0) * (SAFE_DISTANCE - CRITICAL_DISTANCE)

SLOWING_DECCELLERATION = 50#100 # power units / second
SPEED_ACCELERATION = 40#100 # power units / second

BUFFER_DISTANCE = 30 # cm

STOP_THRESHOLD = 0.01

SAMPLE_SIZE = 10#20     # number of uss readings to sample for relative velocity
ALERT_THRESHOLD = 5.0 # 0.01

USS_ERROR = "USS_ERROR"
NOTHING_FOUND = "NOTHING_FOUND"

class ACC(object):
    def __init__(self, system_info, command_queue, user_set_speed, safe_distance):
        """
        Initializes the rover object and sets the values based on the given
        parameters and the current state of the rover.

        :param multiprocessing.Queue command_queue: The queue that the rover
            will pull commands from.
        :param float user_set_speed: The user set speed. None will cause a
            reasonable default to be used.
        :param int safe_distance: The user set safe distance. None will cause a
            reasonable default to be used.
        """
        self.system_info = system_info
        self.command_queue = command_queue

        if user_set_speed is None:
            motor_speeds = gopigo.read_motor_speed()
            self.user_set_speed = (motor_speeds[0] + motor_speeds[1]) / 2.0
        else:
            self.user_set_speed = user_set_speed

        if safe_distance is None:
            self.safe_distance = 2 * BUFFER_DISTANCE
        else:
            self.safe_distance = safe_distance

        self.initial_ticks_left = 0
        self.initial_ticks_right = 0

        self.elapsed_ticks_left = 0
        self.elapsed_ticks_right = 0

        self.speed = INITIAL_SPEED

        self.power_on = False

        self.obstacle_distance = None
        self.obstacle_relative_speed = None

        self.alert_distance = 0

        self.t = 0

        self.dists = collections.deque(maxlen=SAMPLE_SIZE)
        self.dts = collections.deque(maxlen=SAMPLE_SIZE - 1)

        # TODO: More

    def __update_system_info(self):
        self.system_info.setCurrentSpeed(self.speed)
        self.system_info.setObstacleDistance(self.obstacle_distance)
        self.system_info.setTicksLeft(self.elapsed_ticks_left)
        self.system_info.setTicksRight(self.elapsed_ticks_right)

        self.system_info.setUserSetSpeed(self.user_set_speed)
        self.system_info.setSafeDistance(self.safe_distance)
        self.system_info.setAlertDistance(self.alert_distance)

        self.system_info.setPower(self.power_on)

        # TODO: Add more values

    def run(self):
        """
        Starts the acc to control the rover.
        """
        self.__power_on()

        self.__main()

    def __power_on(self):
        self.power_on = True

        gopigo.trim_write(TRIM)

        # Print out the battery voltage so that we can make sure the that
        # the batteries are not low
        time.sleep(0.1)
        print("Volt: " + str(gopigo.volt()))

        time.sleep(0.1)
        self.initial_ticks_left = gopigo.enc_read(gopigo.LEFT)
        time.sleep(0.1)
        self.initial_ticks_right = gopigo.enc_read(gopigo.RIGHT)

        print("Initial\tL: " + str(self.initial_ticks_left) + "\tR: " + \
            str(self.initial_ticks_right))

    def __process_commands(self):
        if not self.command_queue.empty():
            command = self.command_queue.get()
            if isinstance(command, commands.ChangeSettingsCommand):
                if command.userSetSpeed is not None:
                    self.user_set_speed = command.userSetSpeed
                else:
                    motor_speeds = gopigo.read_motor_speed()
                    self.user_set_speed = (motor_speeds[0] + motor_speeds[1]) / 2.0

                if command.safeDistance is not None:
                    self.safe_distance = command.safeDistance
                else:
                    self.safe_distance = gopigo.us_dist(gopigo.USS)

            if isinstance(command, commands.TurnOffCommand):
                self.power_on = False

    def __observe_obstacle(self, dt):
        self.obstacle_distance = get_dist()
        print("Dist: " + str(self.obstacle_distance))

        if not isinstance(self.obstacle_distance, str):
            self.dists.append(float(self.obstacle_distance))
            self.dts.append(float(dt))

        self.obstacle_relative_speed = None
        if len(self.dists) > 9:
            self.obstacle_relative_speed = calculate_relative_speed(self.dists, self.dts)
            print("Rel speed: " + str(self.obstacle_relative_speed))

    def __stop_until_safe_distance(self):
        gopigo.stop()
        self.obstacle_distance = get_dist()
        while (isinstance(self.obstacle_distance, str) and \
            self.obstacle_distance != NOTHING_FOUND) or \
            self.obstacle_distance < self.safe_distance:
            self.obstacle_distance = get_dist()

        gopigo.set_speed(0)
        gopigo.fwd()

    def __handle_alert_distance(self, dt):
        """
        Determines the new speed of the rover when it is in the alert distance
        in order to attempt to match the speed of the obstacle.

        Keeps going at a minimum speed even if the obstacle is not moving so that
        it will stop around the safe distance.

        :param float dt: The change in time for the previous run of the main loop (s)
        :return: The new speed of the rover (power units)
        :rtype: float
        """
        if self.obstacle_relative_speed > ALERT_THRESHOLD:
            print("Alert speeding")
            new_speed = self.speed + dt * SPEED_ACCELERATION

            return new_speed
        elif self.obstacle_relative_speed < -ALERT_THRESHOLD:
            print("Alert slowing")
            new_speed = self.speed - dt * SLOWING_DECCELLERATION

            if new_speed < MIN_SPEED:
                new_speed = MIN_SPEED

            return new_speed
        else:
            print("Alert stable")
            return self.speed

    def __calculate_relevant_distances(self):
        self.alert_distance = self.safe_distance * ALERT_DISTANCE_CONST
        pass # TODO

    def __validate_user_settings(self):
        pass # TODO

    def __obstacle_based_acceleration_determination(self, dt):
        if (isinstance(self.obstacle_distance, str) and \
            self.obstacle_distance != NOTHING_FOUND) or \
            self.obstacle_distance <= CRITICAL_DISTANCE:
            print("<= Critical")
            self.__stop_until_safe_distance()
            self.speed = 0
            self.t = time.time()
        elif self.obstacle_distance <= self.safe_distance:
            print("<= Safe")
            if self.speed > STOP_THRESHOLD:
                #speed = speed - dt * SPEED_DECCELLERATION
                self.speed = self.speed - dt * get_deccelleration(self.speed)
            else:
                self.speed = 0
        elif self.speed > self.user_set_speed:
            print("Slowing down")
            self.speed = self.speed - dt * SLOWING_DECCELLERATION
        elif self.obstacle_distance <= self.alert_distance and \
            self.obstacle_relative_speed is not None:
            self.speed = self.__handle_alert_distance(dt)
        elif self.speed < self.user_set_speed:
            print("Speeding up")
            self.speed = self.speed + dt * SPEED_ACCELERATION
            #speed = speed - dt * get_deccelleration(speed)


    def __straightness_correction(self):
        """
        Returns the power adjustments to make to each motor to correct the
        straightness of the path of the rover.

        :return: The left and right motor speed adjustments (power units)
        :rtype: tuple[float, float]
        """
        self.elapsed_ticks_left, self.elapsed_ticks_right = \
            read_enc_ticks(self.initial_ticks_left, self.initial_ticks_right)

        print("L: " + str(self.elapsed_ticks_left) + "\tR: " + str(self.elapsed_ticks_right))

        # Handle invalid encoder readings
        if self.elapsed_ticks_left < 0 and self.elapsed_ticks_right < 0:
            print("Bad encoder reading")
            return (0, 0)
        if self.elapsed_ticks_left > self.elapsed_ticks_right:
            print("Right slow")
            return (-get_inc(self.speed), get_inc(self.speed))
        elif self.elapsed_ticks_left < self.elapsed_ticks_right:
            print("Left slow")
            return (get_inc(self.speed), -get_inc(self.speed))
        else:
            print("Equal")
            return (0, 0)

    def __actualize_power(self, l_diff, r_diff):
        if self.speed >= MIN_SPEED:
            gopigo.set_left_speed(int(self.speed + l_diff))
            gopigo.set_right_speed(int(self.speed + r_diff))
        else:
            gopigo.set_left_speed(0)
            gopigo.set_right_speed(0)

    def __main(self):
        #print("Critical: " + str(CRITICAL_DISTANCE))
        #print("Safe:     " + str(self.safe_distance))
        #print("Alert:    " + str(self.alert_distance))


        try:
            gopigo.set_speed(0)
            gopigo.fwd()

            self.t = time.time()
            while self.power_on:
                self.__update_system_info()

                if self.speed < 0:
                    self.speed = 0

                print("========================")
                self.__process_commands()

                dt = time.time() - self.t
                self.t = time.time()

                print("Time: " + str(dt))

                self.__observe_obstacle(dt)
                self.__calculate_relevant_distances()

                self.__validate_user_settings()

                self.__obstacle_based_acceleration_determination(dt)

                l_diff, r_diff = self.__straightness_correction()

                self.__actualize_power(l_diff, r_diff)

                print("Speed: " + str(self.speed))

        except (KeyboardInterrupt, Exception):
            traceback.print_exc()
            gopigo.stop()
        gopigo.stop()

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

def read_enc_ticks(initial_ticks_left, initial_ticks_right):
    time.sleep(0.01)
    elapsed_ticks_left = gopigo.enc_read(gopigo.LEFT) - initial_ticks_left
    #time.sleep(0.005)
    time.sleep(0.01)
    elapsed_ticks_right = gopigo.enc_read(gopigo.RIGHT) - initial_ticks_right
    #time.sleep(0.005)

    return (elapsed_ticks_left, elapsed_ticks_right)

def calculate_relative_speed(dists, dts):
    old_dist = sum(list(dists)[0:len(dists) / 2]) / (len(dists) / 2)
    new_dist = sum(list(dists)[len(dists) / 2:]) / (len(dists) / 2)

    avg_dt = sum(list(dts)) / len(dts)

    rel_speed = (new_dist - old_dist) / avg_dt

    return rel_speed

def get_dist():
    time.sleep(0.01)
    dist = gopigo.us_dist(gopigo.USS)

    if dist == -1:
        return USS_ERROR
    elif dist == 0 or dist == 1:
        return NOTHING_FOUND
    else:
        return dist
