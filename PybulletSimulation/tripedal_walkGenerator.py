import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import math

import csv
import os

import tripedal_kinematics


class WalkGenerator():
    def __init__(self):
        super().__init__()

        self._motorDirectionRight = np.array(
            [+1, +1, +1, +1, +1, +1 + 1, +1, +1])
        '''
        self._walkPointX0
        self._walkPointX1
        self._walkPointX2
        self._walkPointX3
        self._walkPointX4
        self._walkPointX5
        '''

    def SetWalkParameter(self,
                         moveDirection,
                         bodyMovePoint,
                         legMovePoint,
                         stepLength,
                         stepHeight,
                         legXYDistanceFromCenter,
                         sit,
                         sway,
                         swayShift,
                         swayRadiusMin,
                         swayRadiusMax,
                         liftPush=0.4,
                         landPull=0.6,
                         damping=0,
                         incline=0):
        self._moveDirection = moveDirection  # angle of direction. leg a direction is 0. leg B direction is pi * 2/3
        self._bodyMovePoint = bodyMovePoint
        self._legMovePoint = legMovePoint
        self._l = stepLength
        self._h = stepHeight
        self._legToCenter = legXYDistanceFromCenter
        self._sit = sit
        self._sway = sway
        self._swayShift = swayShift
        self._liftPush = liftPush  # push the lifting foot backward when lifting the foot to gains momentum.
        self._landPull = landPull  # Before put the foot down, go forward more and pull back when landing.
        self._damping = damping  # damping at the start and end of foot lift.
        self._incline = incline  # tangent angle of incline
        self._swayRadiusMin = swayRadiusMin
        self._swayRadiusMax = swayRadiusMax

    def MakePointListAll(self, generationCommand='FromInitialPoint'):

        walkPoint = self._bodyMovePoint * 3 + self._legMovePoint * 3
        trajectoryLength = self._l * (
            3 * self._bodyMovePoint + 2 * self._legMovePoint) / (
                2 * self._bodyMovePoint + 2 * self._legMovePoint)

        walkPointStep = np.zeros((self._legMovePoint, 3))
        walkPointBodyMove = np.zeros(
            (self._bodyMovePoint * 3 + self._legMovePoint * 2, 3))

        # walking motion
        for i in range(self._legMovePoint):
            t = (i + 1) / self._legMovePoint
            sin_tpi = math.sin(t * math.pi)
            x = (2 * t - 1 + (1 - t) * self._liftPush * -sin_tpi +
                 t * self._landPull * sin_tpi) * trajectoryLength / 2
            y = 0
            walkPointStep[i][0] = math.cos(self._moveDirection) * x - math.sin(
                self._moveDirection) * y
            walkPointStep[i][1] = math.sin(self._moveDirection) * x + math.cos(
                self._moveDirection) * y
            walkPointStep[i][2] = math.sin(t * math.pi) * self._h + self._sit

        for i in range(self._bodyMovePoint * 3 + self._legMovePoint * 2):
            x = -trajectoryLength * (
                (i + 1) /
                (self._bodyMovePoint * 3 + self._legMovePoint * 2) - 0.5)
            y = 0

            walkPointBodyMove[i][0] = math.cos(
                self._moveDirection) * x - math.sin(self._moveDirection) * y
            walkPointBodyMove[i][1] = math.sin(
                self._moveDirection) * x + math.cos(self._moveDirection) * y
            walkPointBodyMove[i][2] = self._sit

        # fig = plt.figure(1)
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(walkPointStep[:,0], walkPointStep[:,1], walkPointStep[:,2], 'o')
        # ax.plot(walkPointBodyMove[:,0], walkPointBodyMove[:,1], walkPointBodyMove[:,2], 'o')
        # plt.show()

        self._walkPointX0 = walkPointBodyMove[self._bodyMovePoint * 2 +
                                              self._legMovePoint *
                                              2:self._bodyMovePoint * 3 +
                                              self._legMovePoint * 2]
        self._walkPointX1 = walkPointStep
        self._walkPointX2 = walkPointBodyMove[:self._bodyMovePoint]
        self._walkPointX3 = walkPointBodyMove[self._bodyMovePoint:self.
                                              _bodyMovePoint +
                                              self._legMovePoint]
        self._walkPointX4 = walkPointBodyMove[
            self._bodyMovePoint + self._legMovePoint:self._bodyMovePoint * 2 +
            self._legMovePoint]
        self._walkPointX5 = walkPointBodyMove[
            self._bodyMovePoint * 2 +
            self._legMovePoint:self._bodyMovePoint * 2 +
            self._legMovePoint * 2]

        self._swayPlusLegA = np.zeros(
            (self._legMovePoint + self._bodyMovePoint, 3))
        self._swayPlusLegB = np.zeros(
            (self._legMovePoint + self._bodyMovePoint, 3))
        self._swayPlusLegC = np.zeros(
            (self._legMovePoint + self._bodyMovePoint, 3))

        for i in range(self._legMovePoint + self._bodyMovePoint):
            t = (i + 1) / (self._legMovePoint + self._bodyMovePoint)  # 0 ~ 1
            self._swayPlusLegA[i][0] = -self._swayRadiusMin / 2 - math.sin(
                math.pi * t) * (self._swayRadiusMax - self._swayRadiusMin / 2)
            self._swayPlusLegA[i][
                1] = math.sqrt(3) * self._swayRadiusMin / 2 - t * math.sqrt(
                    3) * self._swayRadiusMin - math.sin(math.pi * 2 * t) * (
                        (self._swayRadiusMax -
                         (0.5 + 3 / math.pi) * self._swayRadiusMin) /
                        (2 * math.sqrt(3)))
            self._swayPlusLegA[i][2] = 0

            rotation = math.pi * 2 / 3
            self._swayPlusLegB[i][0] = math.cos(rotation) * self._swayPlusLegA[
                i][0] - math.sin(rotation) * self._swayPlusLegA[i][1]
            self._swayPlusLegB[i][1] = math.sin(rotation) * self._swayPlusLegA[
                i][0] + math.cos(rotation) * self._swayPlusLegA[i][1]
            self._swayPlusLegB[i][2] = 0

            rotation = -math.pi * 2 / 3
            self._swayPlusLegC[i][0] = math.cos(rotation) * self._swayPlusLegA[
                i][0] - math.sin(rotation) * self._swayPlusLegA[i][1]
            self._swayPlusLegC[i][1] = math.sin(rotation) * self._swayPlusLegA[
                i][0] + math.cos(rotation) * self._swayPlusLegA[i][1]
            self._swayPlusLegC[i][2] = 0

        swayPlusCABCA = np.vstack([
            self._swayPlusLegC, self._swayPlusLegA, self._swayPlusLegB,
            self._swayPlusLegC, self._swayPlusLegA
        ])

        # fig = plt.figure(1)
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(self._swayPlusLegA[:,0], self._swayPlusLegA[:,1], self._swayPlusLegA[:,2], 'o')
        # ax.plot(self._swayPlusLegB[:,0], self._swayPlusLegB[:,1], self._swayPlusLegB[:,2], 'o')
        # ax.plot(self._swayPlusLegC[:,0], self._swayPlusLegC[:,1], self._swayPlusLegC[:,2], 'o')
        # plt.show()

        self._SwayPlus0 = swayPlusCABCA[
            self._swayShift + self._bodyMovePoint * 0 +
            self._legMovePoint * 1:self._swayShift + self._bodyMovePoint * 1 +
            self._legMovePoint * 1, :]
        self._SwayPlus1 = swayPlusCABCA[
            self._swayShift + self._bodyMovePoint * 1 +
            self._legMovePoint * 1:self._swayShift + self._bodyMovePoint * 1 +
            self._legMovePoint * 2, :]
        self._SwayPlus2 = swayPlusCABCA[
            self._swayShift + self._bodyMovePoint * 1 +
            self._legMovePoint * 2:self._swayShift + self._bodyMovePoint * 2 +
            self._legMovePoint * 2, :]
        self._SwayPlus3 = swayPlusCABCA[
            self._swayShift + self._bodyMovePoint * 2 +
            self._legMovePoint * 2:self._swayShift + self._bodyMovePoint * 2 +
            self._legMovePoint * 3, :]
        self._SwayPlus4 = swayPlusCABCA[
            self._swayShift + self._bodyMovePoint * 2 +
            self._legMovePoint * 3:self._swayShift + self._bodyMovePoint * 3 +
            self._legMovePoint * 3, :]
        self._SwayPlus5 = swayPlusCABCA[
            self._swayShift + self._bodyMovePoint * 3 +
            self._legMovePoint * 3:self._swayShift + self._bodyMovePoint * 3 +
            self._legMovePoint * 4, :]

        GroundPointA = np.array([self._legToCenter, 0, 0])

        rotation = math.pi * 2 / 3
        rx = math.cos(rotation) * self._legToCenter
        ry = math.sin(rotation) * self._legToCenter
        GroundPointB = np.array([rx, ry, 0])

        rotation = -math.pi * 2 / 3
        rx = math.cos(rotation) * self._legToCenter
        ry = math.sin(rotation) * self._legToCenter
        GroundPointC = np.array([rx, ry, 0])

        self._walkPointA0 = self._walkPointX0 - self._SwayPlus0 + GroundPointA
        self._walkPointA1 = self._walkPointX1 - self._SwayPlus1 + GroundPointA
        self._walkPointA2 = self._walkPointX2 - self._SwayPlus2 + GroundPointA
        self._walkPointA3 = self._walkPointX3 - self._SwayPlus3 + GroundPointA
        self._walkPointA4 = self._walkPointX4 - self._SwayPlus4 + GroundPointA
        self._walkPointA5 = self._walkPointX5 - self._SwayPlus5 + GroundPointA

        self._walkPointB0 = self._walkPointX4 - self._SwayPlus0 + GroundPointB
        self._walkPointB1 = self._walkPointX5 - self._SwayPlus1 + GroundPointB
        self._walkPointB2 = self._walkPointX0 - self._SwayPlus2 + GroundPointB
        self._walkPointB3 = self._walkPointX1 - self._SwayPlus3 + GroundPointB
        self._walkPointB4 = self._walkPointX2 - self._SwayPlus4 + GroundPointB
        self._walkPointB5 = self._walkPointX3 - self._SwayPlus5 + GroundPointB

        self._walkPointC0 = self._walkPointX2 - self._SwayPlus0 + GroundPointC
        self._walkPointC1 = self._walkPointX3 - self._SwayPlus1 + GroundPointC
        self._walkPointC2 = self._walkPointX4 - self._SwayPlus2 + GroundPointC
        self._walkPointC3 = self._walkPointX5 - self._SwayPlus3 + GroundPointC
        self._walkPointC4 = self._walkPointX0 - self._SwayPlus4 + GroundPointC
        self._walkPointC5 = self._walkPointX1 - self._SwayPlus5 + GroundPointC

        self._SwayPointA0 = self._SwayPlus0 + GroundPointA + self._walkPointX0[
            0]
        self._SwayPointA1 = self._SwayPlus1 + GroundPointA + self._walkPointX0[
            0]
        self._SwayPointA2 = self._SwayPlus2 + GroundPointA + self._walkPointX0[
            0]
        self._SwayPointA3 = self._SwayPlus3 + GroundPointA + self._walkPointX0[
            0]
        self._SwayPointA4 = self._SwayPlus4 + GroundPointA + self._walkPointX0[
            0]
        self._SwayPointA5 = self._SwayPlus5 + GroundPointA + self._walkPointX0[
            0]

        self._SwayPointB0 = self._SwayPlus0 + GroundPointB + self._walkPointX4[
            0]
        self._SwayPointB1 = self._SwayPlus1 + GroundPointB + self._walkPointX4[
            0]
        self._SwayPointB2 = self._SwayPlus2 + GroundPointB + self._walkPointX4[
            0]
        self._SwayPointB3 = self._SwayPlus3 + GroundPointB + self._walkPointX4[
            0]
        self._SwayPointB4 = self._SwayPlus4 + GroundPointB + self._walkPointX4[
            0]
        self._SwayPointB5 = self._SwayPlus5 + GroundPointB + self._walkPointX4[
            0]

        self._SwayPointC0 = self._SwayPlus0 + GroundPointC + self._walkPointX2[
            0]
        self._SwayPointC1 = self._SwayPlus1 + GroundPointC + self._walkPointX2[
            0]
        self._SwayPointC2 = self._SwayPlus2 + GroundPointC + self._walkPointX2[
            0]
        self._SwayPointC3 = self._SwayPlus3 + GroundPointC + self._walkPointX2[
            0]
        self._SwayPointC4 = self._SwayPlus4 + GroundPointC + self._walkPointX2[
            0]
        self._SwayPointC5 = self._SwayPlus5 + GroundPointC + self._walkPointX2[
            0]

    def ShowGaitPoint3D(self):
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(self._walkPointX0[:, 0], self._walkPointX0[:, 1],
                self._walkPointX0[:, 2], 'o')
        ax.plot(self._walkPointX1[:, 0], self._walkPointX1[:, 1],
                self._walkPointX1[:, 2], '>')
        ax.plot(self._walkPointX2[:, 0], self._walkPointX2[:, 1],
                self._walkPointX2[:, 2], 'o')
        ax.plot(self._walkPointX3[:, 0], self._walkPointX3[:, 1],
                self._walkPointX3[:, 2], 'o')
        ax.plot(self._walkPointX4[:, 0], self._walkPointX4[:, 1],
                self._walkPointX4[:, 2], 'o')
        ax.plot(self._walkPointX5[:, 0], self._walkPointX5[:, 1],
                self._walkPointX5[:, 2], 'o')
        plt.show()

        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self._SwayPlus0[:, 0],
                self._SwayPlus0[:, 1],
                self._SwayPlus0[:, 2],
                'o',
                label='a0')
        ax.plot(self._SwayPlus1[:, 0],
                self._SwayPlus1[:, 1],
                self._SwayPlus1[:, 2],
                '>',
                label='a1')
        ax.plot(self._SwayPlus2[:, 0],
                self._SwayPlus2[:, 1],
                self._SwayPlus2[:, 2],
                'o',
                label='a2')
        ax.plot(self._SwayPlus3[:, 0],
                self._SwayPlus3[:, 1],
                self._SwayPlus3[:, 2],
                '<',
                label='a3')
        ax.plot(self._SwayPlus4[:, 0],
                self._SwayPlus4[:, 1],
                self._SwayPlus4[:, 2],
                'o',
                label='a4')
        ax.plot(self._SwayPlus5[:, 0],
                self._SwayPlus5[:, 1],
                self._SwayPlus5[:, 2],
                '*',
                label='a5')
        plt.show()

        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self._walkPointA0[:, 0], self._walkPointA0[:, 1],
                self._walkPointA0[:, 2], 'o')
        ax.plot(self._walkPointA1[:, 0], self._walkPointA1[:, 1],
                self._walkPointA1[:, 2], '>')
        ax.plot(self._walkPointA2[:, 0], self._walkPointA2[:, 1],
                self._walkPointA2[:, 2], 'o')
        ax.plot(self._walkPointA3[:, 0], self._walkPointA3[:, 1],
                self._walkPointA3[:, 2], 'o')
        ax.plot(self._walkPointA4[:, 0], self._walkPointA4[:, 1],
                self._walkPointA4[:, 2], 'o')
        ax.plot(self._walkPointA5[:, 0], self._walkPointA5[:, 1],
                self._walkPointA5[:, 2], 'o')

        ax.plot(self._walkPointB0[:, 0], self._walkPointB0[:, 1],
                self._walkPointB0[:, 2], 'o')
        ax.plot(self._walkPointB1[:, 0], self._walkPointB1[:, 1],
                self._walkPointB1[:, 2], 'o')
        ax.plot(self._walkPointB2[:, 0], self._walkPointB2[:, 1],
                self._walkPointB2[:, 2], 'o')
        ax.plot(self._walkPointB3[:, 0], self._walkPointB3[:, 1],
                self._walkPointB3[:, 2], '<')
        ax.plot(self._walkPointB4[:, 0], self._walkPointB4[:, 1],
                self._walkPointB4[:, 2], 'o')
        ax.plot(self._walkPointB5[:, 0], self._walkPointB5[:, 1],
                self._walkPointB5[:, 2], 'o')

        ax.plot(self._walkPointC0[:, 0], self._walkPointC0[:, 1],
                self._walkPointC0[:, 2], 'o')
        ax.plot(self._walkPointC1[:, 0], self._walkPointC1[:, 1],
                self._walkPointC1[:, 2], 'o')
        ax.plot(self._walkPointC2[:, 0], self._walkPointC2[:, 1],
                self._walkPointC2[:, 2], 'o')
        ax.plot(self._walkPointC3[:, 0], self._walkPointC3[:, 1],
                self._walkPointC3[:, 2], 'o')
        ax.plot(self._walkPointC4[:, 0], self._walkPointC4[:, 1],
                self._walkPointC4[:, 2], 'o')
        ax.plot(self._walkPointC5[:, 0], self._walkPointC5[:, 1],
                self._walkPointC5[:, 2], '*')
        plt.show()


def test():
    wg = WalkGenerator()
    wg.SetWalkParameter(moveDirection=0,
                        bodyMovePoint=10,
                        legMovePoint=7,
                        stepLength=50,
                        stepHeight=1,
                        legXYDistanceFromCenter=80,
                        sit=60,
                        sway=0,
                        swayShift=5,
                        swayRadiusMin=20,
                        swayRadiusMax=26,
                        liftPush=0.6,
                        landPull=0.6,
                        damping=0,
                        incline=0)
    wg.MakePointListAll()
    wg.ShowGaitPoint3D()


def SaveCSV(moveDirection=0,
            bodyMovePoint=10,
            legMovePoint=7,
            stepLength=50,
            stepHeight=1,
            legXYDistanceFromCenter=80,
            sit=60,
            sway=0,
            swayShift=5,
            swayRadiusMin=20,
            swayRadiusMax=26,
            liftPush=0.6,
            landPull=0.6,
            damping=0,
            incline=0):
    wg = WalkGenerator()
    wg.SetWalkParameter(moveDirection, bodyMovePoint, legMovePoint, stepLength,
                        stepHeight, legXYDistanceFromCenter, sit, sway,
                        swayShift, swayRadiusMin, swayRadiusMax, liftPush,
                        landPull, damping, incline)
    wg.MakePointListAll()
    kinematicTool = tripedal_kinematics.TripedalKinematics()

    file = open('motion23.csv', 'w', encoding='utf-8', newline='')
    csvfile = csv.writer(file)

    csvfile.writerow(
        [1] + kinematicTool.GetMotorDegreeAnglesFromGroundCenterPoints(
            wg._walkPointA0[0], wg._walkPointB0[0], wg._walkPointC0[0]) +
        [1000, 0])
    for i in range(wg._bodyMovePoint):
        csvfile.writerow(
            [1] + kinematicTool.GetMotorDegreeAnglesFromGroundCenterPoints(
                wg._walkPointA0[i], wg._walkPointB0[i], wg._walkPointC0[i]) +
            [60, 0])
    for i in range(wg._legMovePoint):
        csvfile.writerow(
            [1] + kinematicTool.GetMotorDegreeAnglesFromGroundCenterPoints(
                wg._walkPointA1[i], wg._walkPointB1[i], wg._walkPointC1[i]) +
            [60, 0])
    for i in range(wg._bodyMovePoint):
        csvfile.writerow(
            [1] + kinematicTool.GetMotorDegreeAnglesFromGroundCenterPoints(
                wg._walkPointA2[i], wg._walkPointB2[i], wg._walkPointC2[i]) +
            [60, 0])
    for i in range(wg._legMovePoint):
        csvfile.writerow(
            [1] + kinematicTool.GetMotorDegreeAnglesFromGroundCenterPoints(
                wg._walkPointA3[i], wg._walkPointB3[i], wg._walkPointC3[i]) +
            [60, 0])
    for i in range(wg._bodyMovePoint):
        csvfile.writerow(
            [1] + kinematicTool.GetMotorDegreeAnglesFromGroundCenterPoints(
                wg._walkPointA4[i], wg._walkPointB4[i], wg._walkPointC4[i]) +
            [60, 0])
    for i in range(wg._legMovePoint):
        csvfile.writerow(
            [1] + kinematicTool.GetMotorDegreeAnglesFromGroundCenterPoints(
                wg._walkPointA5[i], wg._walkPointB5[i], wg._walkPointC5[i]) +
            [60, 0])

    file.close()

    wg.ShowGaitPoint3D()


if __name__ == "__main__":
    test()