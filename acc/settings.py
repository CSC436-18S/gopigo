class SystemInfo():
    def __init__(self):
        self.currentSpeed = 0
        self.obstacleDistance = 9999
        self.ticksLeft = 0
        self.ticksRight = 0

        self.userSetSpeed = 0
        self.safeDistance = 9999
        self.criticalDistance = 0
        self.alertDistance = 0

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

    def getTicksLeft(self):
        return self.ticksLeft

    def getTicksRight(self):
        return self.ticksRight

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

    def setTicksLeft(self, ticks):
        self.ticksLeft = ticks

    def setTicksRight(self, ticks):
        self.ticksRight = ticks
