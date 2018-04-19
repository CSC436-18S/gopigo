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
MIN_SPEED = 30          # The minimum speed to allow the rover to go at. This prevents issues caused by mechanical
                        # differences between the motors that cause problems at very low speeds. (power units)

INC_CONST = 10          # The power difference to apply to compensate for the rover not moving in a straight path. (power units)

CRITICAL_DISTANCE_MIN = 12  # The minimum critical distance to allow. This should be sufficient to allow the rover to
                            # stop at most speeds. (cm)

BUFFER_DISTANCE = 10            # A distance from the critical distance to use in calculating the minimum settable safe
                                # distance (cm)

TIMESTEPS_TO_APPROACH_SD = 20   # A number of runs of the main loop to use in calculating various values relating to
                                # moving towards the safe distance (timesteps)

SLOWING_DECELERATION = 50       # A deceleration to apply to the speed in order to slow down (power unit/s^2)
SPEED_ACCELERATION = 40         # An acceleration to apply to the speed in order to speed up (power unit/s^2)

STOP_THRESHOLD = 0.01       # A value used to prevent odd fluctuations of the speed when between the critical and safe
                            # distances (power units)

SAMPLE_SIZE = 10            # The number of obstacle distance readings to sample to calculate relative velocity
ALERT_THRESHOLD = 5.0       # A threshold used to determine whether to adjust speed based on the obstacle's relative
                            # velocity, when between the safe and alert distances (cm/s)

ENCODER_READ_TRIES = 5      # The number of tries to allow for repeated encoder tick readings to try to get a valid set
                            # of readings.

USS_ERROR = "USS_ERROR"
NOTHING_FOUND = "NOTHING_FOUND"


class ACC(object):
    """
    A class used for representing and running the ACC.
    """

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
        self.system_info = system_info          # An object that relays ACC, rover, and obstacle information to the user
                                                # interface
        self.command_queue = command_queue      # A concurrent queue that will contain commands for the ACC for changing
                                                # settings and shutting it down

        self.user_set_speed = None              # The speed that the user desires the rover to move at (power units)
        if user_set_speed is None:
            motor_speeds = gopigo.read_motor_speed()
            self.user_set_speed = (motor_speeds[0] + motor_speeds[1]) / 2.0
        else:
            self.user_set_speed = user_set_speed

        self.safe_distance = None               # The distance that the user desires the rover to maintain from the obstacle (cm)
        if safe_distance is None:
            self.safe_distance = 2 * BUFFER_DISTANCE
        else:
            self.safe_distance = safe_distance

        self.initial_ticks_left = 0             # The number of left motor ticks when the ACC was started (ticks)
        self.initial_ticks_right = 0            # The number of right motor ticks when the ACC was started (ticks)

        self.elapsed_ticks_left = 0             # The number of left motor ticks elapsed since the ACC was started (ticks)
        self.elapsed_ticks_right = 0            # The number of right motor ticks elapsed since the ACC was started (ticks)

        self.speed = INITIAL_SPEED              # The speed that the rover is/should be going at (power units)

        self.power_on = False                   # A value representing if the ACC is on or not

        self.obstacle_distance = None           # The distance of the obstacle from the rover (cm)
        self.obstacle_relative_speed = None     # The velocity of the obstacle relative to the rover (cm/s)

        self.critical_distance = 0                  # The critical distance of the rover to the obstacle (cm)
        self.minimum_settable_safe_distance = 0     # The minimum settable safe distance (cm)
        self.alert_distance = 0                     # The alert distance of the rover to the obstacle (cm)

        self.t = 0                              # A value used for calculating the delta time for the main loop (s)

        self.dists = collections.deque(maxlen=SAMPLE_SIZE)      # A log of previous obstacle distance measures (cm)
        self.dts = collections.deque(maxlen=SAMPLE_SIZE - 1)    # A log of previous delta times for the main loop (s)

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
        self.system_info.setSafeDistance(int(self.safe_distance))
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
            new_speed = self.speed - dt * SLOWING_DECELERATION

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

        if self.obstacle_relative_speed is not None:
            self.alert_distance = self.safe_distance + TIMESTEPS_TO_APPROACH_SD * dt * abs(self.obstacle_relative_speed)
        else:
            self.alert_distance = self.safe_distance + TIMESTEPS_TO_APPROACH_SD * dt * self.speed

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
            if self.speed > STOP_THRESHOLD:
                self.speed = self.speed - dt * self.__get_deceleration()
            else:
                self.speed = 0
        elif self.speed > self.user_set_speed:
            self.system_info.setSafetyRange("Slowing")
            self.speed = self.speed - dt * SLOWING_DECELERATION
        elif self.obstacle_distance <= self.alert_distance and \
            self.obstacle_relative_speed is not None:
            self.system_info.setSafetyRange("Alert")

            self.speed = self.__handle_alert_distance(dt)
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

        if self.elapsed_ticks_left < 0 or self.elapsed_ticks_right < 0:
            return 0, 0     # Handle invalid encoder readings
        if self.elapsed_ticks_left > self.elapsed_ticks_right:
            return -get_inc(self.speed), get_inc(self.speed)
        elif self.elapsed_ticks_left < self.elapsed_ticks_right:
            return get_inc(self.speed), -get_inc(self.speed)
        else:
            return 0, 0

    def __actualize_power(self, l_diff, r_diff):
        """
        Applies the decided motor powers to the motors of the rover.

        If the desired motor power is below the minimum allowed motor power,
        then the motors are stopped to prevent issues with motor differences at
        low motor powers.

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

    :param float speed: The current speed that the rover is going (power units)
    :return: The correction power change (power units)
    :rtype: float
    """
    if speed < MIN_SPEED:
        return 0
    else:
        return INC_CONST


def read_enc_ticks(initial_ticks_left, initial_ticks_right):
    """
    Reads the encoder ticks from the left and right motors and returns the
    number of ticks that have occurred from each since the start of the ACC.

    Uses time.sleep calls to try to reduce issues with reading and writing to
    pins to communicate with the rover, as the official gopigo library does
    this.

    If an error is returned from either of the encoder readings, then it tries
    to take more readings until either a valid pair of readings is made or a
    try limit is exceeded.

    :param int initial_ticks_left: The number of left motor ticks recorded at
    the start of the ACC. (ticks)
    :param int initial_ticks_right: The number of right motor ticks recorded at
    the start of the ACC. (ticks)
    :return: The number of ticks for the left and right motor that have
    occurred since the start of the ACC. (ticks)
    :rtype: tuple[int, int]
    """
    found_good_reading = False
    tries = 0
    while not found_good_reading and tries < ENCODER_READ_TRIES:
        time.sleep(0.01)
        elapsed_ticks_left = gopigo.enc_read(gopigo.LEFT) - initial_ticks_left
        time.sleep(0.01)
        elapsed_ticks_right = gopigo.enc_read(gopigo.RIGHT) - initial_ticks_right

        tries += 1

        if elapsed_ticks_left >= 0 and elapsed_ticks_right >= 0:
            found_good_reading = True

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
