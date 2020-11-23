from matplotlib.pyplot import plot
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import os
import csv

from tripedal_kinematics import TripedalKinematics


COS120 = math.cos(math.pi * 2 / 3)
SIN120 = math.sin(math.pi * 2 / 3)
COS240 = math.cos(-math.pi * 2 / 3)
SIN240 = math.sin(-math.pi * 2 / 3)


class WalkGenerator():
    def __init__(self):
        super().__init__()

    def SetWalkParameter(self,
                         moveDirection: float,
                         bodyMovePointsCount: int,
                         legMovePointsCount: int,
                         stepLength: float,
                         stepHeight: float,
                         legXYDistanceFromCenter: float,
                         sit: float,
                         swayShift: int,
                         swayRadiusMin: float,
                         swayRadiusMax: float,
                         liftPush=0.4,
                         landPull=0.6,
                         damping=0,
                         incline=0):
        # I recommend adjusting these values while checking the graph.
        # This is not an algorithm created through any research, it is just an implementation of my idea.

        self._moveDirection = moveDirection  # angle of direction. leg a direction is 0. leg B direction is pi * 2/3
        #                                      발을 움직이는 각
        self._bodyMoveCount = bodyMovePointsCount  # number of points when the legs are not floated.
        #                                            3점 지지 (세 발이 다 바닥에 붙어있고 몸체가 움직임) 때 지점의 갯수
        self._legMoveCount = legMovePointsCount  # number of points when one leg is floating
        #                                            발이 움직일때 지점의 갯수.
        self._l = stepLength  # The distance of one step.
        #                      보폭
        self._h = stepHeight  # The height of the step.
        #                       발을 들어올리는 높이. 모션을 만든 후에 (실시간 센서 값과 별다른 알고리즘 등을 통하여)
        #                       값을 조절하여 쓸 것이라면 높이를 설정하여도 무방하지만 이 코드대로 쓸 것이면 0 값을 추천함.
        self._legToCenter = legXYDistanceFromCenter  # The value of how far the foot is from the central axis.
        #                                             If 0, the three feet are clustered in the middle (of course not recommended).
        #                                             Increasing the value increases the distance of three feet.
        #                                             중심 축으로부터 발이 얼만큼 떨어져 있는지 값임.
        #                                             0이면 세 발이 가운데 모여있음 (당연히 권장 안함). 값을 증가시킬수록 세 발의 거리가 벌려짐.
        self._sit = sit  #
        #
        self._swayShift = swayShift  # Adjust the timing of sway and foot lift.
        #                             If this value is 1, the foot is lifted when the body is swayed to the maximum
        #                             opposite direction of the moving foot.
        #                             If this value is 0, the foot is floated when the body is swayed maximum.
        #                             around 0.5 is recommended. do not set under -0.5 and over 1.5
        #                             이 값이 0이면 발이 뜰때, 1이면 발이 착지할 때 몸체가 최대로 sway 된다. 0.5 주변의 값을 추천함.

        self._liftPush = liftPush  # push the lifting foot backward when lifting the foot to gains momentum. 0.2 ~ 1.0 is recommended.
        #                            이 값을 증가시키면 발 들어올린 직후의 약간 발을 뒤로 함. 0이면 완전한 사인 곡선 형태로 움직임.
        #                            증가시킬수록 둥글게 됨. 0.2~1.0의 값을 추천함.
        self._landPull = landPull  # Before put the foot down, go forward more and pull back when landing.
        #                            이 값을 증가시키면 발을 착륙하기 직전에 발을 앞으로 함. 0이면 완전한 사인 곡선
        #                            형태로 착륙함. 증가시킬수록 둥글게 됨. 0.2~1.0의 값을 추천함.

        self._swayRadiusMin = swayRadiusMin  # minimum length to sway
        self._swayRadiusMax = swayRadiusMax  # maximum length to sway in the opposite direction of the moving foot.

        self._damping = damping  #  // not implemented yet
        self._incline = incline  # tangent angle of incline // not implemented yet
        #버리게 될 거:
        '''
        self._swayRadiusMin
        self._swayRadiusMax
        '''

    def InitProperty(self):
        cosMov = math.cos(self._moveDirection)
        sinMov = math.sin(self._moveDirection)

        self._InitialPointA = [self._legToCenter, 0, 0]
        rx = COS120 * self._legToCenter
        ry = SIN120 * self._legToCenter
        self._InitialPointB = [rx, ry, 0]
        rx = COS240 * self._legToCenter
        ry = SIN240 * self._legToCenter
        self._InitialPointC = [rx, ry, 0]

        self._cycleLength = (self._bodyMoveCount * 3 + self._legMoveCount * 3)
        self._cycleCount = int(0)
        self._notWalkPoitCount = (self._bodyMoveCount * 3 + self._legMoveCount * 2)

        self._liftedVectorA = [0.0, 0.0, 0.0]
        self._liftedVectorB = [0.0, 0.0, 0.0]
        self._liftedVectorC = [0.0, 0.0, 0.0]

        self._puttedVectorA = [0.0, 0.0, 0.0]
        self._puttedVectorB = [0.0, 0.0, 0.0]
        self._puttedVectorC = [0.0, 0.0, 0.0]

        self._targetToPutVectorA = [0.0, 0.0, 0.0]
        self._targetToPutVectorB = [0.0, 0.0, 0.0]
        self._targetToPutVectorC = [0.0, 0.0, 0.0]

        self._moveVectorA = [0.0, 0.0, 0.0]
        self._moveVectorB = [0.0, 0.0, 0.0]
        self._moveVectorC = [0.0, 0.0, 0.0]

        self._resultVectorA = [0.0, 0.0, 0.0]
        self._resultVectorB = [0.0, 0.0, 0.0]
        self._resultVectorC = [0.0, 0.0, 0.0]

        self._swayVector = [0.0, 0.0, self._sit]

        self._swayLength = 0.0
        self._dragVectorChangeSpeed = 0
        self._dragVectorChangeSpeedMax = 3.0
        self._dragVectorChanged = False
        self._dragVectorMult = (3 * self._bodyMoveCount + 2 * self._legMoveCount) / (2 * self._bodyMoveCount +
                                                                                     2 * self._legMoveCount)
        self._dragVectorX = 0.0
        self._dragVectorY = 0.0
        self._dragVectorX_target = 0.0
        self._dragVectorY_target = 0.0

    def MakeNextPoint(self):
        isThreeSupport = None  # bool
        progThreeSupport = None  # float
        progFloatX = None  # float

        progDragA = None  # float
        progDragB = None  # float
        progDragC = None  # float

        FloatingLegVectorX = None  # float
        FloatingLegVectorZ = None  # float

        i = self._cycleCount % (self._legMoveCount + self._bodyMoveCount)

        if i >= self._bodyMoveCount:
            # foot lift
            progFloatX = (i + 1 - self._bodyMoveCount) / self._legMoveCount  # 0 ~ 1
            isThreeSupport = False
        else:
            # three foots suport
            progThreeSupport = (i + 1) / self._bodyMoveCount  # 0 ~ 1
            isThreeSupport = True

        cyclecountA = (self._cycleCount + self._bodyMoveCount * 2 + self._legMoveCount * 2) % self._cycleLength
        cyclecountB = (self._cycleCount + self._bodyMoveCount + self._legMoveCount) % self._cycleLength
        cyclecountC = (self._cycleCount) % self._cycleLength

        cosMov = math.cos(self._moveDirection)
        sinMov = math.sin(self._moveDirection)

        difX = self._dragVectorX_target - self._dragVectorX
        difY = self._dragVectorY_target - self._dragVectorY
        distXY = math.sqrt(difX * difX + difY * difY)

        if (isThreeSupport == True):
            if progThreeSupport < 0.7 and progThreeSupport > 0.3:
                dragVecX = -self._l * self._dragVectorMult * cosMov
                dragVecY = -self._l * self._dragVectorMult * sinMov
                # if target drag vector changed
                if (self._dragVectorX_target != dragVecX or self._dragVectorY_target != dragVecY):
                    #change target drag vector
                    self._dragVectorX_target = -self._l * self._dragVectorMult * cosMov
                    self._dragVectorY_target = -self._l * self._dragVectorMult * sinMov

                    difX = self._dragVectorX_target - self._dragVectorX
                    difY = self._dragVectorY_target - self._dragVectorY
                    distXY = math.sqrt(difX * difX + difY * difY)
                    if (distXY > 0):
                        count = math.ceil(
                            (distXY / self._dragVectorChangeSpeedMax) / (self._bodyMoveCount + self._legMoveCount))
                        self._dragVectorChangeSpeed = distXY / ((self._bodyMoveCount + self._legMoveCount) * count)
                    else:
                        self._dragVectorChangeSpeed = 0.0

        else:
            t = progFloatX  # 0 ~ 1
            sin_tpi = math.sin(t * math.pi)
            x = (2 * t + (1 - t) * self._liftPush * -sin_tpi + t * self._landPull * sin_tpi) / 2  #0~1
            FloatingLegVectorX = x
            FloatingLegVectorZ = sin_tpi * self._h

        if (distXY > 0):
            if (distXY >= self._dragVectorChangeSpeed):
                if difX != 0.0:
                    vecx = self._dragVectorChangeSpeed * difX / distXY
                    self._dragVectorX = self._dragVectorX + vecx
                if difY != 0.0:
                    vecy = self._dragVectorChangeSpeed * difY / distXY
                    self._dragVectorY = self._dragVectorY + vecy
            else:
                self._dragVectorX = self._dragVectorX_target
                self._dragVectorY = self._dragVectorY_target

        # A
        if (cyclecountA < self._notWalkPoitCount):
            # drag
            progDragA = float(cyclecountA + 1) / self._notWalkPoitCount

            self._moveVectorA[0] = self._moveVectorA[0] + self._dragVectorX / self._notWalkPoitCount
            self._moveVectorA[1] = self._moveVectorA[1] + self._dragVectorY / self._notWalkPoitCount
            self._moveVectorA[2] = self._sit

            if (progDragA == 1.0):
                # ready to float
                self._liftedVectorA[0] = self._moveVectorA[0]
                self._liftedVectorA[1] = self._moveVectorA[1]

                self._targetToPutVectorA[0] = -(self._dragVectorX / 2)
                self._targetToPutVectorA[1] = -(self._dragVectorY / 2)
        else:
            # float
            progFloatA = progFloatX

            self._moveVectorA[0] = self._targetToPutVectorA[0] * FloatingLegVectorX + (
                1 - FloatingLegVectorX) * self._liftedVectorA[0]
            self._moveVectorA[1] = self._targetToPutVectorA[1] * FloatingLegVectorX + (
                1 - FloatingLegVectorX) * self._liftedVectorA[1]
            self._moveVectorA[2] = self._sit + FloatingLegVectorZ

            if (progFloatX == 1.0):
                # put
                self._puttedVectorA[0] = self._moveVectorA[0]
                self._puttedVectorA[1] = self._moveVectorA[1]
                self._puttedVectorA[2] = self._moveVectorA[2]

        # B
        if (cyclecountB < self._notWalkPoitCount):
            # drag
            progDragB = float(cyclecountB + 1) / self._notWalkPoitCount

            self._moveVectorB[0] = self._moveVectorB[0] + self._dragVectorX / self._notWalkPoitCount
            self._moveVectorB[1] = self._moveVectorB[1] + self._dragVectorY / self._notWalkPoitCount
            self._moveVectorB[2] = self._sit

            if (progDragB == 1.0):
                # ready to float
                self._liftedVectorB[0] = self._moveVectorB[0]
                self._liftedVectorB[1] = self._moveVectorB[1]

                self._targetToPutVectorB[0] = -(self._dragVectorX / 2)
                self._targetToPutVectorB[1] = -(self._dragVectorY / 2)
        else:
            # float
            progFloatB = progFloatX

            self._moveVectorB[0] = self._targetToPutVectorB[0] * FloatingLegVectorX + (
                1 - FloatingLegVectorX) * self._liftedVectorB[0]
            self._moveVectorB[1] = self._targetToPutVectorB[1] * FloatingLegVectorX + (
                1 - FloatingLegVectorX) * self._liftedVectorB[1]
            self._moveVectorB[2] = self._sit + FloatingLegVectorZ

            if (progFloatX == 1.0):
                # put
                self._puttedVectorB[0] = self._moveVectorB[0]
                self._puttedVectorB[1] = self._moveVectorB[1]
                self._puttedVectorB[2] = self._moveVectorB[2]

        # C
        if (cyclecountC < self._notWalkPoitCount):
            # drag
            progDragC = float(cyclecountC + 1) / self._notWalkPoitCount

            self._moveVectorC[0] = self._moveVectorC[0] + self._dragVectorX / self._notWalkPoitCount
            self._moveVectorC[1] = self._moveVectorC[1] + self._dragVectorY / self._notWalkPoitCount
            self._moveVectorC[2] = self._sit

            if (progDragC == 1.0):
                # ready to float
                self._liftedVectorC[0] = self._moveVectorC[0]
                self._liftedVectorC[1] = self._moveVectorC[1]

                self._targetToPutVectorC[0] = -(self._dragVectorX / 2)
                self._targetToPutVectorC[1] = -(self._dragVectorY / 2)
        else:
            # float
            progFloatC = progFloatX

            self._moveVectorC[0] = self._targetToPutVectorC[0] * FloatingLegVectorX + (
                1 - FloatingLegVectorX) * self._liftedVectorC[0]
            self._moveVectorC[1] = self._targetToPutVectorC[1] * FloatingLegVectorX + (
                1 - FloatingLegVectorX) * self._liftedVectorC[1]
            self._moveVectorC[2] = self._sit + FloatingLegVectorZ

            if (progFloatX == 1.0):
                # put
                self._puttedVectorC[0] = self._moveVectorC[0]
                self._puttedVectorC[1] = self._moveVectorC[1]
                self._puttedVectorC[2] = self._moveVectorC[2]

        # sway vector
        i = self._cycleCount % (self._legMoveCount + self._bodyMoveCount)
        t = -0.5 + (i + 1 + self._swayShift * self._legMoveCount) / (self._legMoveCount + self._bodyMoveCount)

        if t < 0:
            t = t + 1
            tmpX = -self._swayRadiusMin / 2 - math.sin(math.pi * t) * (self._swayRadiusMax - self._swayRadiusMin / 2)
            tmpY = math.sqrt(3) * self._swayRadiusMin / 2 - t * math.sqrt(3) * self._swayRadiusMin - math.sin(
                math.pi * 2 * t) * ((self._swayRadiusMax - (0.5 + 3 / math.pi) * self._swayRadiusMin) /
                                    (2 * math.sqrt(3)))
            #rotation = -math.pi * 2 / 3
            tmpSwayAX = COS240 * tmpX - SIN240 * tmpY
            tmpSwayAY = SIN240 * tmpX + COS240 * tmpY
        elif t > 1:
            t = t - 1
            tmpX = -self._swayRadiusMin / 2 - math.sin(math.pi * t) * (self._swayRadiusMax - self._swayRadiusMin / 2)
            tmpY = math.sqrt(3) * self._swayRadiusMin / 2 - t * math.sqrt(3) * self._swayRadiusMin - math.sin(
                math.pi * 2 * t) * ((self._swayRadiusMax - (0.5 + 3 / math.pi) * self._swayRadiusMin) /
                                    (2 * math.sqrt(3)))
            #rotation = math.pi * 2 / 3
            tmpSwayAX = COS120 * tmpX - SIN120 * tmpY
            tmpSwayAY = SIN120 * tmpX + COS120 * tmpY
        else:
            tmpSwayAX = -self._swayRadiusMin / 2 - math.sin(
                math.pi * t) * (self._swayRadiusMax - self._swayRadiusMin / 2)
            tmpSwayAY = math.sqrt(3) * self._swayRadiusMin / 2 - t * math.sqrt(3) * self._swayRadiusMin - math.sin(
                math.pi * 2 * t) * ((self._swayRadiusMax - (0.5 + 3 / math.pi) * self._swayRadiusMin) /
                                    (2 * math.sqrt(3)))

        #0 before_A_move
        if self._cycleCount < self._bodyMoveCount * 1 + self._legMoveCount * 1:
            self._swayVector = np.array([tmpSwayAX, tmpSwayAY, 0])
        #2 before_B_move
        elif self._cycleCount < self._bodyMoveCount * 2 + self._legMoveCount * 2:
            self._swayVector = np.array(
                [COS120 * tmpSwayAX - SIN120 * tmpSwayAY, SIN120 * tmpSwayAX + COS120 * tmpSwayAY, 0])
        #4 before_C_move
        else:
            self._swayVector = np.array(
                [COS240 * tmpSwayAX - SIN240 * tmpSwayAY, SIN240 * tmpSwayAX + COS240 * tmpSwayAY, 0])

        self._resultVectorA[0] = self._moveVectorA[0] + self._InitialPointA[0] - self._swayVector[0]
        self._resultVectorA[1] = self._moveVectorA[1] + self._InitialPointA[1] - self._swayVector[1]
        self._resultVectorA[2] = self._moveVectorA[2] + self._InitialPointA[2] - 0
        self._resultVectorB[0] = self._moveVectorB[0] + self._InitialPointB[0] - self._swayVector[0]
        self._resultVectorB[1] = self._moveVectorB[1] + self._InitialPointB[1] - self._swayVector[1]
        self._resultVectorB[2] = self._moveVectorB[2] + self._InitialPointB[2] - 0
        self._resultVectorC[0] = self._moveVectorC[0] + self._InitialPointC[0] - self._swayVector[0]
        self._resultVectorC[1] = self._moveVectorC[1] + self._InitialPointC[1] - self._swayVector[1]
        self._resultVectorC[2] = self._moveVectorC[2] + self._InitialPointC[2] - 0

        self._cycleCount = self._cycleCount + 1
        if self._cycleCount >= self._cycleLength:
            self._cycleCount = 0

        return self._resultVectorA, self._resultVectorB, self._resultVectorC, self._swayVector


def ShowGraphAnimation():
    wg = WalkGenerator()
    wg.SetWalkParameter(moveDirection=0,
                        bodyMovePointsCount=10,
                        legMovePointsCount=7,
                        stepLength=50,
                        stepHeight=30,
                        legXYDistanceFromCenter=70,
                        sit=60,
                        swayShift=0.5,
                        swayRadiusMin=20,
                        swayRadiusMax=26,
                        liftPush=0.6,
                        landPull=0.6,
                        damping=0,
                        incline=0)

    #wg.BeforeMakeOnePoint()
    wg.InitProperty()
    queue = []

    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-80, 80])
    ax.set_ylim([-80, 80])
    ax.set_proj_type('ortho')

    dv, = ax.plot(wg._dragVectorX / 2, wg._dragVectorY / 2, float(wg._sit), 'x', color='red')
    dvt, = ax.plot(wg._dragVectorX_target / 2, wg._dragVectorY_target / 2, float(wg._sit), '+', color='purple')

    plt.pause(1)

    for _ in range(wg._cycleLength - 10):
        a, b, c, s = wg.MakeNextPoint()
        aa, = ax.plot(a[0], a[1], a[2], '>', color='red')
        bb, = ax.plot(b[0], b[1], b[2], '^', color='green')
        cc, = ax.plot(c[0], c[1], c[2], 'v', color='blue')
        ss, = ax.plot(s[0], s[1], wg._sit, '*', color='yellow')
        queue.append(aa)
        queue.append(bb)
        queue.append(cc)
        queue.append(ss)
        plt.pause(0.001)

    for _ in range(40):
        a, b, c, s = wg.MakeNextPoint()
        aa, = ax.plot(a[0], a[1], a[2], '>', color='red')
        bb, = ax.plot(b[0], b[1], b[2], '^', color='green')
        cc, = ax.plot(c[0], c[1], c[2], 'v', color='blue')
        ss, = ax.plot(s[0], s[1], wg._sit, '*', color='yellow')
        queue.append(aa)
        queue.append(bb)
        queue.append(cc)
        queue.append(ss)
        da = queue.pop(0)
        db = queue.pop(0)
        dc = queue.pop(0)
        ds = queue.pop(0)
        da.remove()
        db.remove()
        dc.remove()
        ds.remove()

        dv.remove()
        dv, = ax.plot(wg._dragVectorX / 2, wg._dragVectorY / 2, float(wg._sit), 'x', color='red')
        dvt.remove()
        dvt, = ax.plot(wg._dragVectorX_target / 2, wg._dragVectorY_target / 2, float(wg._sit), '+', color='purple')

        plt.pause(0.001)

    wg._l = 30
    wg._moveDirection = 1
    print('''
    wg._l = 30
    wg._moveDirection = 1''')

    #while (True):
    for _ in range(150):
        a, b, c, s = wg.MakeNextPoint()
        aa, = ax.plot(a[0], a[1], a[2], '>', color='red')
        bb, = ax.plot(b[0], b[1], b[2], '^', color='green')
        cc, = ax.plot(c[0], c[1], c[2], 'v', color='blue')
        ss, = ax.plot(s[0], s[1], wg._sit, '*', color='yellow')
        queue.append(aa)
        queue.append(bb)
        queue.append(cc)
        queue.append(ss)
        da = queue.pop(0)
        db = queue.pop(0)
        dc = queue.pop(0)
        ds = queue.pop(0)
        da.remove()
        db.remove()
        dc.remove()
        ds.remove()

        dv.remove()
        dv, = ax.plot(wg._dragVectorX / 2, wg._dragVectorY / 2, float(wg._sit), 'x', color='red')
        dvt.remove()
        dvt, = ax.plot(wg._dragVectorX_target / 2, wg._dragVectorY_target / 2, float(wg._sit), '+', color='purple')

        plt.pause(0.001)

    wg._l = 40
    wg._moveDirection = -1.5
    print('''
    wg._l = 40
    wg._moveDirection = -1.5''')

    #while (True):
    for _ in range(150):
        a, b, c, s = wg.MakeNextPoint()
        aa, = ax.plot(a[0], a[1], a[2], '>', color='red')
        bb, = ax.plot(b[0], b[1], b[2], '^', color='green')
        cc, = ax.plot(c[0], c[1], c[2], 'v', color='blue')
        ss, = ax.plot(s[0], s[1], wg._sit, '*', color='yellow')
        queue.append(aa)
        queue.append(bb)
        queue.append(cc)
        queue.append(ss)
        da = queue.pop(0)
        db = queue.pop(0)
        dc = queue.pop(0)
        ds = queue.pop(0)
        da.remove()
        db.remove()
        dc.remove()
        ds.remove()

        dv.remove()
        dv, = ax.plot(wg._dragVectorX / 2, wg._dragVectorY / 2, float(wg._sit), 'x', color='red')
        dvt.remove()
        dvt, = ax.plot(wg._dragVectorX_target / 2, wg._dragVectorY_target / 2, float(wg._sit), '+', color='purple')

        plt.pause(0.001)

    wg._l = 0
    wg._moveDirection = -1.5
    print('''
    wg._l = 40
    wg._moveDirection = -1.5''')

    #while (True):
    for _ in range(150):
        a, b, c, s = wg.MakeNextPoint()
        aa, = ax.plot(a[0], a[1], a[2], '>', color='red')
        bb, = ax.plot(b[0], b[1], b[2], '^', color='green')
        cc, = ax.plot(c[0], c[1], c[2], 'v', color='blue')
        ss, = ax.plot(s[0], s[1], wg._sit, '*', color='yellow')
        queue.append(aa)
        queue.append(bb)
        queue.append(cc)
        queue.append(ss)
        da = queue.pop(0)
        db = queue.pop(0)
        dc = queue.pop(0)
        ds = queue.pop(0)
        da.remove()
        db.remove()
        dc.remove()
        ds.remove()

        dv.remove()
        dv, = ax.plot(wg._dragVectorX / 2, wg._dragVectorY / 2, float(wg._sit), 'x', color='red')
        dvt.remove()
        dvt, = ax.plot(wg._dragVectorX_target / 2, wg._dragVectorY_target / 2, float(wg._sit), '+', color='purple')

        plt.pause(0.001)


def SaveCSV():
    wg = WalkGenerator()
    wg.SetWalkParameter(moveDirection=0.5,
                        bodyMovePointsCount=10,
                        legMovePointsCount=7,
                        stepLength=50,
                        stepHeight=1,
                        legXYDistanceFromCenter=86,
                        sit=60,
                        swayShift=0.6,
                        swayRadiusMin=18,
                        swayRadiusMax=24,
                        liftPush=0.6,
                        landPull=0.6,
                        damping=0,
                        incline=0)
    wg.InitProperty()

    file = open(os.path.abspath(os.path.dirname(__file__)) + '/TripedalGaitPoints.csv',
                'w',
                encoding='utf-8',
                newline='')
    csvfile = csv.writer(file)
    for i in range(102):
        a, b, c, _ = wg.MakeNextPoint()
    for i in range(51):
        a, b, c, _ = wg.MakeNextPoint()
        csvfile.writerow(a + b + c)

    file.close()


if __name__ == "__main__":
    ShowGraphAnimation()
