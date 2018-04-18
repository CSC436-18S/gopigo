"""
This is the module of the program that contains all of the functionality
related to the ACC's control of the rover.
"""

import gopigo
import commands
import math
import time


BUFFER_DISATANCE = 30               # Approximate length of one Rover, in centimeters.
MAX_TICK_COUNT = 20000              # Number at which the total number of elapsed ticks should be reset
MAX_STOPPING_DISTANCE = 7.5         # Maximum distance the Rovers were found to skid on a slick surface at Max Speed
SLOWDOWN_TIME = 1.0                 # Time desired to decelerate when approaching an obstacle to the necessary speed
MAX_POWER_VALUE = 255               # Maximum Power Value capable of being used for the motors
MAX_SPEED = 99999999                # Self explanatory

WHEEL_RAD = 3.25
WHEEL_CIRC = 2*math.pi*WHEEL_RAD
TICK_MARKS = 18


class ACC:

    def __init__(self, command_queue, user_set_speed, safe_distance):
        """
        :param command_queue: This parameter is created in the "Power On" sequence, in the initial __main__() method
        :param user_set_speed: This parameter is provided by the initial __main__() method, with the value determined
                                as outlined in the "Power On" sequence diagram.
        :param safe_distance: This parameter is provided by the initial __main__() method, with the value determined
                                as outlined in the "Power On" sequence diagram.

        This function sets class parameters utilized in indicating the User's settings. In the case that a
        user_set_speed and safe_distance are not provided, they are set to default values.
        """
        if user_set_speed is None:
            motor_speeds = gopigo.read_motor_speed()
            self.user_set_speed = (motor_speeds[0] + motor_speeds[1]) / 2.0
        else:
            self.user_set_speed = user_set_speed

        if safe_distance is None:
            self.safe_distance = 2*BUFFER_DISATANCE
        else:
            self.safe_distance = safe_distance

        self.command_queue = command_queue
        self.power_on = True
        self.stop = False
        self.elapsed_ticks_left = gopigo.enc_read(gopigo.LEFT)
        self.elapsed_ticks_right = gopigo.enc_read(gopigo.RIGHT)
        self.alert_distance = None
        self.critical_distance = None
        self.current_acceleration = None
        self.current_speed_left = None                  # This is a velocity
        self.current_speed_right = None                 # This is a velocity
        self.safe_speed = None
        self.left_power = 0
        self.right_power = 0
        self.left_rotation_rate = None                  # Ticks per second
        self.right_rotation_rate = None                 # Ticks per second
        self.minimum_settable_safe_distance = None
        self.delta_ticks_l = 0
        self.delta_ticks_r = 0
        self.obstacle_distance = None
        self.obstacle_velocity = None
        self.obstacle_acceleration = None
        self.prev_achieved_v_l = 0
        self.prev_achieved_v_r = 0
        self.set_speed = None
        self.set_distance = None
        self.v_desired_left = None
        self.v_desired_right = None
        self.v_path_left = None
        self.v_path_right = None

    def run(self):
        """
        This function enacts the primary functionality of the ACC, as outlined by the "Main Diagram" sequence diagram.
        """
        print("run")
        prev_time = time.time()
        while self.power_on:
            print("user set speed: " + str(self.user_set_speed))
            self.process_commands()
            if not self.power_on:
                break
            dt = time.time() - prev_time
            print("delta time:\t" + str(dt))
            prev_time = time.time()
            self.determine_safety_values(dt)
            self.validate_user_settings()
            self.straightness_correction_calculation(dt)
            self.velocity_to_power_calculation(dt)
            self.actualize_power()

        #self.power_off(prev_time)


    def process_commands(self):
        """

        :return:
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

            elif isinstance(command, commands.TurnOffCommand):
                self.power_on = False

        #self.print_values()


    def determine_safety_values(self, dt):
        #TODO: May need to determine rotation rates in a more clever manner than simple time differentials
        self.right_rotation_rate = (gopigo.enc_read(gopigo.RIGHT) - self.elapsed_ticks_right)/dt
        self.left_rotation_rate = (gopigo.enc_read(gopigo.LEFT) - self.elapsed_ticks_left)/dt
        
        self.caculate_current_speed()
        self.observe_obstacle(dt)
        self.critical_distance = (MAX_STOPPING_DISTANCE/MAX_POWER_VALUE)*max(self.current_speed_left, self.current_speed_right)
        self.minimum_settable_safe_distance = self.critical_distance + BUFFER_DISATANCE
        self.alert_distance = self.safe_distance + max(self.current_speed_left, self.current_speed_right)*SLOWDOWN_TIME
        self.perform_obstalce_based_acceleratioin_determination(dt)

        #self.print_values()


    def caculate_current_speed(self):
        self.current_speed_left = self.left_rotation_rate * ((2.0*math.pi)/TICK_MARKS) * (WHEEL_CIRC/TICK_MARKS)
        self.current_speed_right = self.right_rotation_rate * ((2.0*math.pi)/TICK_MARKS) * (WHEEL_CIRC/TICK_MARKS)


    def observe_obstacle(self, dt):
        prev_dist = self.obstacle_distance
        self.obstacle_distance = gopigo.us_dist(gopigo.USS)
        #FIXME: This may need re-integrating, depending on how distances are determined for the absence of Obstacles
        #if self.obstacle_distance == 0:
        #    self.obstacle_distance = None
        #else:
        if prev_dist is not None:
            prev_velocity = self.obstacle_velocity
            self.obstacle_velocity = (self.obstacle_distance - prev_dist)/dt + (self.current_speed_right + self.current_speed_left)/2.0
            if prev_velocity is not None:
                self.obstacle_acceleration = (self.obstacle_velocity - prev_velocity)/dt
        else:
            self.obstacle_velocity = None
            self.obstacle_acceleration = None


    def perform_obstalce_based_acceleratioin_determination(self, dt):
        if self.obstacle_distance > self.critical_distance and self.stop:
            self.stop = False

        if self.obstacle_distance <= self.critical_distance:
            self.stop = True
            self.safe_speed = 0
        elif self.obstacle_distance <= self.safe_distance:
            if self.obstacle_acceleration is not None:
                self.current_acceleration = ((self.obstacle_distance - self.safe_distance)/(dt*dt)) + self.obstacle_acceleration
            else:
                self.current_acceleration = (self.obstacle_distance - self.safe_distance)/(dt*dt)
            self.safe_speed = (self.current_speed_left + self.current_speed_right)/2.0 + self.current_acceleration
        elif self.obstacle_distance <= self.alert_distance:
            if self.obstacle_acceleration is not None:
                comfort_parameter = dt/SLOWDOWN_TIME
                self.current_acceleration = \
                    comfort_parameter*(((self.obstacle_distance - self.safe_distance)/(dt*dt)) + self.obstacle_acceleration)
            else:
                comfort_parameter = dt/SLOWDOWN_TIME
                self.current_acceleration = comfort_parameter*((self.obstacle_distance - self.safe_distance)/(dt*dt))
            self.safe_speed = (self.current_speed_left + self.current_speed_right)/2.0 + self.current_acceleration
        else:
            comfort_parameter = dt/SLOWDOWN_TIME
            current_speed = (self.current_speed_left + self.current_speed_right)/2.0
            self.current_acceleration = comfort_parameter*((self.user_set_speed - current_speed)/dt)
            self.safe_speed = MAX_SPEED


    def validate_user_settings(self):
        if self.user_set_speed is not None:
            if self.user_set_speed > self.safe_speed:
              self.power_on = True
            else:
                self.set_speed = self.user_set_speed

        if self.safe_distance is not None:
            if self.safe_distance < self.minimum_settable_safe_distance:
                self.set_distance = self.minimum_settable_safe_distance
            else:
                self.set_distance = self.safe_distance
        
        #self.print_values()


    def straightness_correction_calculation(self, dt):
        if not self.stop:
            self.calculate_elapsed_ticks()
            delta_ticks = abs(self.elapsed_ticks_left - self.elapsed_ticks_right)

            if self.elapsed_ticks_left < self.elapsed_ticks_right:
                self.increase_l_rotation_rate(delta_ticks, dt)
            elif self.elapsed_ticks_right < self.elapsed_ticks_left:
                self.increase_r_rotation_rate(delta_ticks, dt)

        #self.print_values()


    def calculate_elapsed_ticks(self):
        l_ticks = gopigo.enc_read(gopigo.LEFT)
        r_ticks = gopigo.enc_read(gopigo.RIGHT)
        if l_ticks is not None:
            self.delta_ticks_l = l_ticks - self.elapsed_ticks_left
            self.elapsed_ticks_left = self.elapsed_ticks_left + self.delta_ticks_l
        else:
            self.delta_ticks_l = 0

        if r_ticks is not None:
            self.delta_ticks_r = r_ticks - self.elapsed_ticks_right
            self.elapsed_ticks_right = self.elapsed_ticks_right + self.delta_ticks_r
        else:
            self.delta_ticks_r = 0

        #if self.elapsed_ticks_left >= MAX_TICK_COUNT or (self.elapsed_ticks_right >= MAX_TICK_COUNT):
        #    carry_over_delta = self.elapsed_ticks_left - self.elapsed_ticks_right
        #    if carry_over_delta > 0:
        #        self.elapsed_ticks_left = carry_over_delta
        #        self.elapsed_ticks_right = 0
        #    elif carry_over_delta < 0:
        #        self.elapsed_ticks_left = 0
        #        self.elapsed_ticks_right = -carry_over_delta
        #    else:
        #        self.elapsed_ticks_left = 0
        #        self.elapsed_ticks_right = 0


    def increase_l_rotation_rate(self, delta_ticks, dt):
        self.left_rotation_rate = self.left_rotation_rate + (delta_ticks/dt)


    def increase_r_rotation_rate(self, delta_ticks, dt):
        self.right_rotation_rate = self.right_rotation_rate + (delta_ticks/dt)


    def velocity_to_power_calculation(self, dt):
        self.v_path_left = self.left_rotation_rate * ((2.0*math.pi)/TICK_MARKS) * (WHEEL_CIRC/TICK_MARKS)
        self.v_path_right = self.right_rotation_rate * ((2.0*math.pi)/TICK_MARKS) * (WHEEL_CIRC/TICK_MARKS)

        self.v_desired_left = self.v_path_left + self.safe_speed
        self.v_desired_right = self.v_path_right + self.safe_speed

        v_old_l = self.prev_achieved_v_l
        v_old_r = self.prev_achieved_v_r
        self.prev_achieved_v_l = (self.delta_ticks_l/dt) * ((2.0*math.pi)/TICK_MARKS) * (WHEEL_CIRC/TICK_MARKS)
        self.prev_achieved_v_r = (self.delta_ticks_r/dt) * ((2.0*math.pi)/TICK_MARKS) * (WHEEL_CIRC/TICK_MARKS)

        if v_old_l == 0:
            v_new_l = self.v_desired_left
        else:
            v_new_l = (self.prev_achieved_v_l - v_old_l)*(self.left_power/v_old_l)*self.v_desired_left

        if v_old_r == 0:
            v_new_r = self.v_desired_right
        else:
            v_new_r = (self.prev_achieved_v_r - v_old_r)*(self.right_power/v_old_r)*self.v_desired_right

        if self.prev_achieved_v_l != 0:
            self.left_power = self.left_power + (v_new_l*(self.left_power/self.prev_achieved_v_l))

        if self.prev_achieved_v_r != 0:
            self.right_power = self.right_power + (v_new_r*(self.right_power/self.prev_achieved_v_r))

        if self.left_power > MAX_POWER_VALUE:
            self.left_power = MAX_POWER_VALUE

        if self.right_power > MAX_POWER_VALUE:
            self.right_power = MAX_POWER_VALUE

        #self.print_values()


    def actualize_power(self):
        if self.stop:
            gopigo.set_left_speed(0)
            gopigo.set_right_speed(0)
        else:
            if gopigo.enc_read(gopigo.LEFT) == 0 and gopigo.enc_read(gopigo.RIGHT) == 0:
                gopigo.fwd()
            gopigo.set_left_speed(int(self.left_power))
            gopigo.set_right_speed(int(self.right_power))

        self.print_values()


    def power_off(self, prev_time):
        while self.obstacle_distance <= self.minimum_settable_safe_distance:
            self.process_commands()
            dt = time.time() - prev_time
            prev_time = time.time()
            self.determine_safety_values(dt)
            self.validate_user_settings()
            self.straightness_correction_calculation(dt)
            self.velocity_to_power_calculation(dt)

        #self.print_values()


    def print_values(self):
        print("power on:\t" + str(self.power_on))
        print("stop:\t" + str(self.stop))
        print("e.t.l.:\t" + str(self.elapsed_ticks_left))
        print("e.t.l.:\t" + str(self.elapsed_ticks_right))
        print("alert dist:\t" + str(self.alert_distance))
        print("crit. dist:\t" + str(self.critical_distance))
        print("curr. acc:\t" + str(self.current_acceleration))
        print("curr. speed L:\t" + str(self.current_speed_left))
        print("curr. speed R:\t" + str(self.current_speed_right))
        print("safe speed:\t" + str(self.safe_speed))
        print("left power:\t" + str(self.left_power))
        print("right power:\t" + str(self.right_power))
        print("rot. rate L:\t" + str(self.left_rotation_rate))
        print("rot. rate R:\t" + str(self.right_rotation_rate))
        print("min. safe dist.:\t" + str(self.minimum_settable_safe_distance))
        print("delta ticks L:\t" + str(self.delta_ticks_l))
        print("delta ticks R:\t" + str(self.delta_ticks_r))
        print("obs. dist.:\t" + str(self.obstacle_distance))
        print("obs. vel.:\t"  + str(self.obstacle_velocity))
        print("obs. acc.:\t"  + str(self.obstacle_acceleration))
        print("prev. ach. v L:\t" + str(self.prev_achieved_v_l))
        print("prev. ach. v R:\t" + str(self.prev_achieved_v_r))
        print("set speed:\t" + str(self.set_speed))
        print("set dist.:\t" + str(self.set_distance))
        print("desired v L:\t" + str(self.v_desired_left))
        print("desired v R:\t" + str(self.v_desired_right))
        print("v path L:\t" + str(self.v_path_left))
        print("v path R:\t" + str(self.v_path_right))
        
