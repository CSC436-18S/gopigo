"""
This module handles all of the actual functionality of the ACC including
deciding and altering the behaviour of the rover to avoid collisions and
maintain the user_set_speed.
"""
import collections
import math
import time
import traceback

import gopigo

import commands

TRIM = 0                # The trim to apply to the motors of the rover (trim units)
INITIAL_SPEED = 50      # The initial speed to start the rover at (power units)
MAX_SPEED = 250         # The max speed to allow the rover to go at (power units)
MIN_SPEED = 30          # The minimum speed to allow the rover to go at. This
                        # prevents issues caused by mechanical differences
                        # between the motors that cause problems at very low
                        # speeds. (power units)

INC_CONST = 100.0       # A constant used for calculating the power difference
                        # to apply to compensate for the rover not moving in a
                        # straight path.

CRITICAL_DISTANCE_MIN = 12  # The minimum critical distance to allow. This
                            # should be sufficient to allow the rover to stop
                            # at most speeds.

MODE_SAFE_OLD = True
MODE_ALERT_OLD = True

DYNAMIC_ALERT_DISTANCE = True
ALERT_DISTANCE_OFFSET = 40

BUFFER_DISTANCE = 10 # cm
TIMESTEPS_TO_APPROACH_SD = 20

SLOWING_DECCELLERATION = 50#100 # power units / second
SPEED_ACCELERATION = 40#100 # power units / second

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

        self.critical_distance = 0
        self.minimum_settable_safe_distance = 0
        self.alert_distance = 0

        self.t = 0

        self.dists = collections.deque(maxlen=SAMPLE_SIZE)
        self.dts = collections.deque(maxlen=SAMPLE_SIZE - 1)

    def __update_system_info(self):
        """
        Updates the system_info object with information on the state of the
        ACC, rover, and obstacle in order to allow the user to request this
        information through the user interface.
        """
        self.system_info.setCurrentSpeed(self.speed)
        self.system_info.setObstacleDistance(self.obstacle_distance)
        self.system_info.setTicksLeft(self.elapsed_ticks_left)
        self.system_info.setTicksRight(self.elapsed_ticks_right)

        self.system_info.setUserSetSpeed(self.user_set_speed)
        self.system_info.setSafeDistance(self.safe_distance)
        self.system_info.setCriticalDistance(int(self.critical_distance))
        self.system_info.setAlertDistance(int(self.alert_distance))

        self.system_info.setPower(self.power_on)

        if isinstance(self.obstacle_relative_speed, str):
            self.system_info.setObstacleRelSpeed(self.obstacle_relative_speed)
        elif self.obstacle_relative_speed is not None:
            self.system_info.setObstacleRelSpeed(int(math.floor(self.obstacle_relative_speed)))

    def run(self):
        """
        Starts the ACC to control the rover.
        """
        self.__power_on()

        self.__main()

    def __power_on(self):
        """
        Initializes the state of the rover and ACC in order to prepare for the
        ACC to start controlling the rover.

        time.sleep calls are used here to try to reduce the chance of issues
        with writing to and reading from the pins used for interacting with
        the rover. The developers of the official python gopigo library did
        this in, so we figured it would be a good idea to do it in this
        function as it may reduce the chance of bad initial readings while only
        having a one time reduction in the operation time of the ACC.
        """
        self.power_on = True

        # Set the trim value for the rover's motors. Even if the trim value is
        # 0 it should still be set in order to clear out any previous trim
        # setting.
        gopigo.trim_write(TRIM)

        # Get battery voltage in order to make it easy to tell when the battery
        # is getting low, and could thus effect performance of the rover
        time.sleep(0.1)
        volt = gopigo.volt()
        self.system_info.setStartupVoltage(volt)

        # Read the encoder tick amounts at startup to allow all future encoder
        # tick readings to be relative to the rover's state at the startup of
        # the ACC. This allows the ACC to maintain the rover's direction at
        # startup as straight.
        time.sleep(0.1)
        self.initial_ticks_left = gopigo.enc_read(gopigo.LEFT)
        time.sleep(0.1)
        self.initial_ticks_right = gopigo.enc_read(gopigo.RIGHT)

    def __power_to_velocity(self, power):
        """
        Converts a rover motor power value (power units) into an estimated
        velocity (cm/s).

        Formula determined through experiments performed by running the rover
        at various motor powers.

        :param float power: The power value to convert. (power unit)
        :return: The corresponding velocity. (cm/s)
        """
        return float(power) * 0.192

    def __velocity_to_power(self, velocity):
        """
        Converts a velocity (cm/s) into an estimated rover motor power value
        (power units).

        Formula determined through experiments performed by running the rover
        at various motor powers.

        :param float velocity: The velocity to convert. (cm/s)
        :return: The corresponding rover motor power. (power units)
        """
        return float(velocity) / 0.192

    def __process_commands(self):
        """
        Processes the next command in the ACC's command queue if there are any
        queued commands.
        """
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
        """
        Attempts to observe the distance of the obstacle from the rover and
        calculate its relative velocity using previous distance measurements
        and time differences.

        :param float dt: The amount of time it took for the last run of the
        main ACC loop to complete. (s)
        """
        self.obstacle_distance = get_dist()

        if not isinstance(self.obstacle_distance, str):
            self.dists.append(float(self.obstacle_distance))
            self.dts.append(float(dt))

        self.obstacle_relative_speed = None
        if len(self.dists) > 9:
            self.obstacle_relative_speed = calculate_relative_speed(self.dists, self.dts)

    def __stop_until_safe_distance(self):
        """
        Stops the rover until it achieves a safe distance from the obstacle.
        This can occur when/if the obstacle moves away from the rover.
        """
        gopigo.stop()
        self.obstacle_distance = get_dist()
        while (isinstance(self.obstacle_distance, str) and
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
            new_speed = self.speed + dt * SPEED_ACCELERATION

            return new_speed
        elif self.obstacle_relative_speed < -ALERT_THRESHOLD:
            new_speed = self.speed - dt * SLOWING_DECCELLERATION

            if new_speed < MIN_SPEED:
                new_speed = MIN_SPEED

            return new_speed
        else:
            return self.speed

    def __calculate_relevant_distances(self, dt):
        """
        Calculates the dynamically adjusted distance settings used by the ACC.

        :param float dt: The amount of time it took for the last run of the
        main ACC loop to complete. (s)
        """
        self.critical_distance = CRITICAL_DISTANCE_MIN + 7 * (self.speed / float(MAX_SPEED))
        self.minimum_settable_safe_distance = self.critical_distance + BUFFER_DISTANCE

        if DYNAMIC_ALERT_DISTANCE:
            if self.obstacle_relative_speed is not None:
                self.alert_distance = self.safe_distance + TIMESTEPS_TO_APPROACH_SD * dt * abs(self.obstacle_relative_speed)
            else:
                self.alert_distance = self.safe_distance + TIMESTEPS_TO_APPROACH_SD * dt * self.speed
        else:
            self.alert_distance = self.safe_distance + ALERT_DISTANCE_OFFSET

    def __validate_user_settings(self):
        """
        Validates the user settings to make sure that they are possible and
        safe.

        If the settings are impossible or unsafe, then it sets them to safe
        values.
        """
        if self.user_set_speed > MAX_SPEED:
            self.user_set_speed = MAX_SPEED

        if self.safe_distance < self.minimum_settable_safe_distance:
            self.safe_distance = self.minimum_settable_safe_distance

    def __obstacle_based_acceleration_determination(self, dt):
        """
        Determines the speed to set the rover to based on information about the
        obstacle.

        :param float dt: The amount of time it took for the last run of the
        main ACC loop to complete. (s)
        """
        if (isinstance(self.obstacle_distance, str) and
            self.obstacle_distance != NOTHING_FOUND) or \
            self.obstacle_distance <= self.critical_distance:
            self.system_info.setSafetyRange("Critical")
            self.__stop_until_safe_distance()
            self.speed = 0
            self.t = time.time()
        elif self.obstacle_distance <= self.safe_distance:
            self.system_info.setSafetyRange("Safe")
            if MODE_SAFE_OLD:
                if self.speed > STOP_THRESHOLD:
                    self.speed = self.speed - dt * self.__get_deceleration()
                else:
                    self.speed = 0
            else:
                self.speed = self.speed + (self.__velocity_to_power((self.obstacle_distance - self.safe_distance) / dt))
        elif self.speed > self.user_set_speed:
            self.system_info.setSafetyRange("Slowing")
            self.speed = self.speed - dt * SLOWING_DECCELLERATION
        elif self.obstacle_distance <= self.alert_distance and \
            self.obstacle_relative_speed is not None:
            self.system_info.setSafetyRange("Alert")

            if MODE_ALERT_OLD:
                self.speed = self.__handle_alert_distance(dt)
            else:
                acceleration = self.__velocity_to_power(
                    (1.0 / TIMESTEPS_TO_APPROACH_SD) * ((self.alert_distance - self.safe_distance) / (TIMESTEPS_TO_APPROACH_SD * dt)))
                self.speed = self.speed + acceleration
        elif self.speed < self.user_set_speed:
            self.system_info.setSafetyRange("Speeding")
            self.speed = self.speed + dt * SPEED_ACCELERATION
        else:
            self.system_info.setSafetyRange("Maintaining")

    def __straightness_correction(self):
        """
        Returns the power adjustments to make to each motor to correct the
        straightness of the path of the rover.

        If either the encoder value reads yield invalid values then no
        adjustment is attempted.

        :return: The left and right motor speed adjustments (power units)
        :rtype: tuple[float, float]
        """
        self.elapsed_ticks_left, self.elapsed_ticks_right = \
            read_enc_ticks(self.initial_ticks_left, self.initial_ticks_right)

        # Handle invalid encoder readings
        if self.elapsed_ticks_left < 0 or self.elapsed_ticks_right < 0:
            return 0, 0
        if self.elapsed_ticks_left > self.elapsed_ticks_right:
            return -get_inc(self.speed), get_inc(self.speed)
        elif self.elapsed_ticks_left < self.elapsed_ticks_right:
            return get_inc(self.speed), -get_inc(self.speed)
        else:
            return 0, 0

    def __actualize_power(self, l_diff, r_diff):
        """
        Applies the decided motor powers to the motors of the rover.

        :param l_diff: The additional motor power to apply to just the left
        motor. (power units)
        :param r_diff: The additional motor power to apply to just the right
        motor. (power units)
        """
        if self.speed >= MIN_SPEED:
            gopigo.set_left_speed(int(self.speed + l_diff))
            gopigo.set_right_speed(int(self.speed + r_diff))
        else:
            gopigo.set_left_speed(0)
            gopigo.set_right_speed(0)

    def __get_deceleration(self):
        """
        Returns the deceleration amount to use when slowing down when within the
        safe distance.
    
        It is based on the current speed so that at higher speeds it decelerates
        more, and at lower speeds it decelerates less.

        :return: The deceleration to apply to the rover's speed. (power units / seconds)
        :rtype: float
        """
        slowdown_span = (4.0 / 5.0) * (self.safe_distance - self.critical_distance)
        return (self.speed ** 2.0) / (2.0 * slowdown_span)

    def __main(self):
        """
        Runs the main loop of the ACC.

        If an exception is thrown in the main loop or the ACC is killed via
        Ctrl+c then the rover is stopped in order to prevent a collision.
        """
        try:
            gopigo.set_speed(0)
            gopigo.fwd()

            self.t = time.time()
            while self.power_on:
                self.__update_system_info()

                self.__process_commands()

                dt = time.time() - self.t
                self.t = time.time()

                self.__observe_obstacle(dt)
                self.__calculate_relevant_distances(dt)

                self.__validate_user_settings()

                self.__obstacle_based_acceleration_determination(dt)

                # Reset the speed if it becomes negative
                if self.speed < 0:
                    self.speed = 0

                # Reset the speed if it goes below the minimum speed
                if self.user_set_speed < MIN_SPEED:
                    self.speed = 0

                l_diff, r_diff = self.__straightness_correction()

                self.__actualize_power(l_diff, r_diff)

        # Catch any exceptions that occur
        except (KeyboardInterrupt, Exception):
            traceback.print_exc()
            gopigo.stop()           # Stop to prevent a collision
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
        return 2.0 * (9.0 / (speed / INC_CONST)) / 1.5


def read_enc_ticks(initial_ticks_left, initial_ticks_right):
    """
    Reads the encoder ticks from the left and right motors and returns the
    number of ticks that have occurred from each since the start of the ACC.

    Uses time.sleep calls to try to reduce issues with reading and writing to
    pins to communicate with the rover, as the official gopigo library does
    this.

    :param int initial_ticks_left: The number of left motor ticks recorded at
    the start of the ACC. (ticks)
    :param int initial_ticks_right: The number of right motor ticks recorded at
    the start of the ACC. (ticks)
    :return: The number of ticks for the left and right motor that have
    occurred since the start of the ACC. (ticks)
    :rtype: tuple[int, int]
    """
    time.sleep(0.01)
    elapsed_ticks_left = gopigo.enc_read(gopigo.LEFT) - initial_ticks_left
    time.sleep(0.01)
    elapsed_ticks_right = gopigo.enc_read(gopigo.RIGHT) - initial_ticks_right

    return elapsed_ticks_left, elapsed_ticks_right


def calculate_relative_speed(dists, dts):
    """
    Calculates the relative speed of the obstacle to the rover using previously
    recorded obstacle distances and time differences.

    :param collections.deque[int] dists: The previously measured obstacle
    distances. (cm)
    :param collections.deque[float] dts: The previously measured time
    differences. (s)
    :return: The relative speed of the obstacle. (cm/s)
    """
    old_dist = sum(list(dists)[0:len(dists) / 2]) / (len(dists) / 2)
    new_dist = sum(list(dists)[len(dists) / 2:]) / (len(dists) / 2)

    avg_dt = sum(list(dts)) / len(dts)

    rel_speed = (new_dist - old_dist) / ((len(dists) / 2.0 - 1) * avg_dt)

    return rel_speed


def get_dist():
    """
    Measures the distance of the obstacle from the rover.

    Uses a time.sleep call to try to prevent issues with pin writing and
    reading. (See official gopigo library)

    Returns error strings in the cases of measurements of -1 and 0, as -1
    indicates and error, and 0 seems to also indicate a failed reading.

    :return: The distance of the obstacle. (cm)
    :rtype: either[int, str]
    """
    time.sleep(0.01)
    dist = gopigo.us_dist(gopigo.USS)

    if dist == -1:
        return USS_ERROR
    elif dist == 0 or dist == 1:
        return NOTHING_FOUND
    else:
        return dist
