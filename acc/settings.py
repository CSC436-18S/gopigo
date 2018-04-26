class SystemInfo():
    def __init__(self):
        self.userSetSpeed = 0
        self.safeDistance = 9999
        self.currentSpeed = 0
        self.obstacleDistance = 9999
        self.power = True

    def getPower(self):
        return self.power

    def getUserSetSpeed(self):
        return self.userSetSpeed

    def getCurrentSpeed(self):
        return self.currentSpeed

    def getSafeDistance(self):
        return self.safeDistance

    def getObstacleDistance(self):
        return self.obstacleDistance

    def setPower(self, power):
        self.power = power

    def setUserSetSpeed(self, speed):
        self.userSetSpeed = speed

    def setCurrentSpeed(self, speed):
        self.currentSpeed = speed

    def setSafeDistance(self, distance):
        self.safeDistance = distance
    
    def setObstacleDistance(self, distance):
        self.obstacleDistance = distance
