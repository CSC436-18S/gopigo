class TurnOffCommand():
    def __init__(self):
        self.power_off = True


class ChangeSettingsCommand():
    def __init__(self, speed, distance):
        self.userSetSpeed = speed
        self.safeDistance = distance