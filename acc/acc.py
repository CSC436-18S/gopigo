"""
This is the module of the program that contains all of the functionality
related to the ACC's control of the rover.
"""

import gopigo


BUFFER_DISATANCE = 0.3      #Approximate length of one Rover, in meters.
MAX_TICK_COUNT = 20000      #Number at which the total number of elapsed ticks should be reset
WHEEL_RAD=3.25
WHEEL_CIRC=2*math.pi*WHEEL_RAD
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

        if safe_distance is None:
            self.safe_distance = 2*BUFFER_DISATANCE

        self.command_queue = command_queue
        self.power_on = True
        self.stop = False
        self.elapsed_ticks_left = 0
        self.elapsed_ticks_right = 0



    def run(self):
        """
        This function enacts the primary functionality of the ACC, as outlined by the "Main Diagram" sequence diagram.
        """
        while not self.power_on:
            self.process_commands()
            self.determine_safety_values()
            self.validate_user_settings()
            self.straightness_correction_calculation()
            self.velocity_to_power_calculation()
            self.actualize_power()

        self.power_off()


    def process_commands(self):
        """

        :return:
        """


    def determine_safety_values(self):


    def validate_user_settings(self):


    def straightness_correction_calculation(self, dt):
        if not self.stop:
            self.calculate_elapsed_ticks()
            delta_ticks = abs(self.elapsed_ticks_left - self.elapsed_ticks_right)

            if self.elapsed_ticks_left < self.elapsed_ticks_right:
                self.increase_l_rotation_rate(delta_ticks, dt)
            elif self.elapsed_ticks_right > self.elapsed_ticks_left:
                self.increase_r_rotation_rate(delta_ticks, dt)


    def calculate_elapsed_ticks(self):
        l_ticks = gopigo.enc_read(gopigo.LEFT)
        r_ticks = gopigo.enc_read(gopigo.RIGHT)
        if l_ticks is not None:
            old_ticks = self.elapsed_ticks_left
            self.elapsed_ticks_left = l_ticks - self.elapsed_ticks_left
            self.n_ticks_l = self.elapsed_ticks_left - old_ticks
        else:
            self.elapsed_ticks_left = self.elapsed_ticks_left
            self.n_ticks_l = 0

        if r_ticks is not None:
            old_ticks = self.elapsed_ticks_right
            self.elapsed_ticks_right = r_ticks - self.elapsed_ticks_right
            self.n_ticks_r = self.elapsed_ticks_right - old_ticks
        else:
            self.elapsed_ticks_right = self.elapsed_ticks_right
            self.n_ticks_r = 0

        if self.elapsed_ticks_left >= MAX_TICK_COUNT or (self.elapsed_ticks_right >= MAX_TICK_COUNT):
            carry_over_delta = self.elapsed_ticks_left - self.elapsed_ticks_right
            if carry_over_delta > 0:
                self.elapsed_ticks_left = carry_over_delta
                self.elapsed_ticks_right = 0
            elif carry_over_delta < 0:
                self.elapsed_ticks_left = 0
                self.elapsed_ticks_right = -carry_over_delta
            else:
                self.elapsed_ticks_left = 0
                self.elapsed_ticks_right = 0


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
        self.prev_achieved_v_l = (self.n_ticks_l/dt) * ((2.0*math.pi)/TICK_MARKS) * (WHEEL_CIRC/TICK_MARKS)
        self.prev_achieved_v_r = (self.n_ticks_r/dt) * ((2.0*math.pi)/TICK_MARKS) * (WHEEL_CIRC/TICK_MARKS)

        v_new_l = (self.prev_achieved_v_l - v_old_l)*(self.left_power/v_old_l)*self.v_desired_left
        v_new_r = (self.prev_achieved_v_r - v_old_r)*(self.right_power/v_old_r)*self.v_desired_right

        self.left_power = self.left_power + (v_new_l*(self.left_power/self.prev_achieved_v_l))
        self.right_power = self.right_power + (v_new_r*(self.right_power/self.prev_achieved_v_r))

        if self.left_power > 255:
            self.left_power = 255
        elif self.left_power is 255:
            self.left_power = 255

        if self.right_power > 255:
            self.right_power = 255
        elif self.right_power is 255:
            self.right_power = 255


    def actualize_power(self):
        if self.stop:
            gopigo.set_left_speed(0)
            gopigo.set_right_speed(0)
        else:
            gopigo.set_left_speed(self.left_power)
            gopigo.set_right_speed(self.right_power)


    def power_off(self):



