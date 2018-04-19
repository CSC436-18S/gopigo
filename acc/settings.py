UNKNOWN = "-"

class SystemInfo():
    def __init__(self):
        self.currentSpeed = UNKNOWN
        self.obstacleDistance = UNKNOWN
        self.obstacleRelSpeed = UNKNOWN
        self.ticksLeft = UNKNOWN
        self.ticksRight = UNKNOWN
        self.safetyRange = UNKNOWN
        self.startupVoltage = UNKNOWN

        self.userSetSpeed = UNKNOWN
        self.safeDistance = UNKNOWN
        self.criticalDistance = UNKNOWN
        self.alertDistance = UNKNOWN

        self.power = True

    def getPower(self):
        return self.power

    def getUserSetSpeed(self):
        return self.userSetSpeed

    def getCurrentSpeed(self):
        return self.currentSpeed

    def getSafeDistance(self):
        return self.safeDistance

    def getCriticalDistance(self):
        return self.criticalDistance

    def getAlertDistance(self):
        return self.alertDistance

    def getObstacleDistance(self):
        return self.obstacleDistance

    def getObstacleRelSpeed(self):
        return self.obstacleRelSpeed

    def getTicksLeft(self):
        return self.ticksLeft

    def getTicksRight(self):
        return self.ticksRight

    def getSafetyRange(self):
        return self.safetyRange

    def getStartupVoltage(self):
        return self.startupVoltage

    def setPower(self, power):
        self.power = power

    def setUserSetSpeed(self, speed):
        self.userSetSpeed = speed

    def setCurrentSpeed(self, speed):
        self.currentSpeed = int(speed)

    def setSafeDistance(self, distance):
        self.safeDistance = distance

    def setCriticalDistance(self, distance):
        self.criticalDistance = distance

    def setAlertDistance(self, distance):
        self.alertDistance = distance

    def setObstacleDistance(self, distance):
        self.obstacleDistance = distance

    def setObstacleRelSpeed(self, speed):
        self.obstacleRelSpeed = speed

    def setTicksLeft(self, ticks):
        self.ticksLeft = ticks

    def setTicksRight(self, ticks):
        self.ticksRight = ticks

    def setSafetyRange(self, s_range):
        self.safetyRange = s_range

    def setStartupVoltage(self, volt):
        self.startupVoltage = volt
