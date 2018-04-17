"""
This is the module of the program that contains all of the functionality
related to the ACC's control of the rover.
"""

import awefa as gopigo

BUFFER_DISATANCE = 0.3      #Approximate length of one Rover, in meters.

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
        self.stop = False



    def run(self):
        """
        This function enacts the primary functionality of the ACC, as outlined by the "Main Diagram" sequence diagram.
        """
        while not self.stop:
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

    def straightness_correction_calculation(self):

    def velocity_to_power_calculation(self):

    def actualize_power(self):

    def power_off(self):



