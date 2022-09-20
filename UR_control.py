import time

import rtde_control
import rtde_receive
import math
from typing import List

class URLink(object):

    def __init__(self):
        self.IPaddress = "169.254.141.189"
        self.rtde_c = rtde_control.RTDEControlInterface(self.IPaddress)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.IPaddress)

    def moveToSetupPosition(self):
        targetJointPosition = [-3.05, -2.43, 2.64, -2.9, 1.51, 0]
        self.rtde_c.moveJ(targetJointPosition, 0.1, 0.1)

    def moveToReadyPosition(self):
        targetToolPosition = [0.15, 0.13, 0.47, 0, 0, 1.65]
        self.rtde_c.moveL(targetToolPosition, 0.1, 0.1)
        targetToolPosition = [0.27, 0.12, 0.66, 0, 0, 1.61]
        self.rtde_c.moveL(targetToolPosition, 0.1, 0.1)

    def moveToCleanUpPosition(self):
        self.adjustEndRotation(0)
        targetToolPosition = [0.27, 0.10, 0.60, 0, 0, 1.61]
        self.rtde_c.moveL(targetToolPosition, 0.1, 0.1)

    def adjustToolPosition(self, relativeCoordinates: List[float]):
        """For small movements, should not exceed 0.01 in x,y direction, adjust frying pan instead of arm for those cases"""
        if len(relativeCoordinates) != 3:
            raise Exception("input format not accepted!")
        elif abs(relativeCoordinates[0]) > 0.01 or abs(relativeCoordinates[1]) > 0.01:
            raise Exception("input a smaller value!")
        elif abs(relativeCoordinates[2]) > 0.1:
            raise Exception("input a smaller value!")

        currentCoordinates = self.rtde_r.getActualTCPPose()
        relativeCoordinates = relativeCoordinates + 3 * [0]
        targetToolPosition = [currentCoordinates[n] + relativeCoordinates[n] for n in range(6)]
        if targetToolPosition[2] >= 0.741:
            raise Exception("The end of the tool is going too low!")
        self.rtde_c.moveL(targetToolPosition, 0.3, 0.3)

    def adjustEndRotation(self, targetAngle: int):
        """rotate end actuator to specified angle, max 180 in both directions"""
        maxRotation = 180
        currentJointPositions = self.rtde_r.getActualQ()
        targetJointPositions = currentJointPositions
        targetJointPositions[5] = targetAngle / 180 * math.pi
        if abs(targetJointPositions[5]) > maxRotation / 180 * math.pi:
            raise Exception("angle of servo should be within 180")
        self.rtde_c.moveJ(targetJointPositions, 0.5, 0.5)


if __name__ == '__main__':
    URTest = URLink()
    URTest.moveToSetupPosition()
    # URTest.moveToReadyPosition()
    # URTest.adjustToolPosition([0,0,-0.02])
    # time.sleep(10)
    # URTest.adjustToolPosition([0,0,0.02])

    # URTest.adjustEndRotation(0)
    # time.sleep(10)
    # URTest.adjustEndRotation(-90)
    # URTest.moveToCleanUpPosition()