"""
This module contains class for representing commands for the ACC.
"""


class TurnOffCommand(object):
    """
    A command to have the ACC turn off.
    """
    def __init__(self):
        self.power_off = True


class ChangeSettingsCommand(object):
    """
    A command to change the user_set_speed and/or safe distance of the ACC.
    """
    def __init__(self, speed, distance):
        self.userSetSpeed = speed
        self.safeDistance = distance
