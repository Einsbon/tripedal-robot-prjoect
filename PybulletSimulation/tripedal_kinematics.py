import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import math


class TripedalKinematics():
    def __init__(self):
        super().__init__()
        self._StartX = 65.0
        self._StartY = 0.0
        self._StartZ = -20.45

        self._L0 = 21.0
        self._L1 = 90.0
        self._L2 = 90.0
        self._LinkSum = self._L0 + self._L1 + self._L2

        self._GroundCenterPoint = np.array(
            [0, 0, self._StartZ - self._LinkSum])

        self._GroundPointA = np.array(
            [self._StartX, self._StartY, self._StartZ - self._LinkSum])

        rotation = math.pi * 2 / 3
        rx = math.cos(rotation) * self._StartX - math.sin(
            rotation) * self._StartY
        ry = math.sin(rotation) * self._StartX + math.cos(
            rotation) * self._StartY
        self._GroundPointB = np.array([rx, ry, self._StartZ - self._LinkSum])

        rotation = -math.pi * 2 / 3
        rx = math.cos(rotation) * self._StartX - math.sin(
            rotation) * self._StartY
        ry = math.sin(rotation) * self._StartX + math.cos(
            rotation) * self._StartY
        self._GroundPointC = np.array([rx, ry, self._StartZ - self._LinkSum])

    def SetParameter(self, startX, startY, startZ, linkYtoX, linkUpleglength,
                     linkDownLegLength):
        self._StartX = startX
        self._StartY = startY
        self._StartZ = startZ

        self._L0 = linkYtoX
        self._L1 = linkUpleglength
        self._L2 = linkDownLegLength

        self._LinkSum = self._L0 + self._L1 + self._L2

        self._GroundCenterPoint = np.array(
            [0, 0, self._StartZ - self._LinkSum])

        self._GroundPointA = np.array(
            [self._StartX, self._StartY, self._StartZ - self._LinkSum])

        rotation = math.pi * 2 / 3
        rx = math.cos(rotation) * self._StartX - math.sin(
            rotation) * self._StartY
        ry = math.sin(rotation) * self._StartX + math.cos(
            rotation) * self._StartY
        self._GroundPointB = np.array([rx, ry, self._StartZ - self._LinkSum])

        rotation = -math.pi * 2 / 3
        rx = math.cos(rotation) * self._StartX - math.sin(
            rotation) * self._StartY
        ry = math.sin(rotation) * self._StartX + math.cos(
            rotation) * self._StartY
        self._GroundPointC = np.array([rx, ry, self._StartZ - self._LinkSum])

    def GetMotorAnglesFromPointsXYZ(self, xa, ya, za, xb, yb, zb, xc, yc, zc):
        return self.InverseLegA(xa, ya, za) + self.InverseLegB(
            xb, yb, zb) + self.InverseLegC(xc, yc, zc)

    def GetMotorAnglesFromPoints(self, a, b, c):
        return self.InverseLegA(a[0], a[1], a[2]) + self.InverseLegB(
            b[0], b[1], b[2]) + self.InverseLegC(c[0], c[1], c[2])

    def GetMotorAnglesFromInitialPointsXYZ(self, xa, ya, za, xb, yb, zb, xc,
                                           yc, zc):
        return self.InverseLegAFromInitialPoint(
            xa, ya, za) + self.InverseLegBFromInitialPoint(
                xb, yb, zb) + self.InverseLegCFromInitialPoint(xc, yc, zc)

    def GetMotorAnglesFromInitialPoints(self, a, b, c):
        return self.InverseLegAFromInitialPoint(
            a[0], a[1], a[2]) + self.InverseLegBFromInitialPoint(
                b[0], b[1], b[2]) + self.InverseLegCFromInitialPoint(
                    c[0], c[1], c[2])

    def GetMotorAnglesFromGroundCenterPointsXYZ(self, xa, ya, za, xb, yb, zb,
                                                xc, yc, zc):
        return self.InverseLegA(xa, ya, za) + self.InverseLegB(
            xb, yb, zb) + self.InverseLegC(xc, yc, zc)

    def GetMotorAnglesFromGroundCenterPoints(self, a, b, c):
        pointA = a + self._GroundCenterPoint
        pointB = b + self._GroundCenterPoint
        pointC = c + self._GroundCenterPoint
        return self.InverseLegA(
            pointA[0], pointA[1], pointA[2]) + self.InverseLegB(
                pointB[0], pointB[1], pointB[2]) + self.InverseLegC(
                    pointC[0], pointC[1], pointC[2])

    def GetMotorDegreeAnglesFromGroundCenterPoints(self, a, b, c):
        pointA = a + self._GroundCenterPoint
        pointB = b + self._GroundCenterPoint
        pointC = c + self._GroundCenterPoint
        return list(
            np.array(
                self.InverseLegA(pointA[0], pointA[1], pointA[2]) +
                self.InverseLegB(pointB[0], pointB[1], pointB[2]) +
                self.InverseLegC(pointC[0], pointC[1], pointC[2])) * 180 /
            math.pi)

    def InverseLegA(self, x, y, z):
        # Leg 1 inverse kinematics.

        tx = x - self._StartX
        ty = y - self._StartY
        tz = -(z - self._StartZ)

        d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
        d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)
        if (d12 > (self._L1 + self._L2)):
            tx = tx * ((self._L1 + self._L2) / d12)
            ty = ty * ((self._L1 + self._L2) / d12)
            tz = tz * ((self._L1 + self._L2) / d12)
            d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
            d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)

        angle0 = -math.pi / 2 + math.atan2(tz, tx)
        angle1 = math.atan2(ty, d_projXZ) + math.acos(
            (self._L1 * self._L1 + d12 * d12 - self._L2 * self._L2) /
            (2 * self._L1 * d12))
        angle2 = -math.pi + math.acos(
            (self._L1 * self._L1 + self._L2 * self._L2 - d12 * d12) /
            (2 * self._L1 * self._L2))

        return [angle0, angle1, angle2]

    def InverseLegAFromInitialPoint(self, x, y, z):
        # Leg 1 inverse kinematics.

        tx = x
        ty = y
        tz = self._L0 + self._L1 + self._L2 - z

        d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
        d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)
        if (d12 > (self._L1 + self._L2)):
            tx = tx * ((self._L1 + self._L2) / d12)
            ty = ty * ((self._L1 + self._L2) / d12)
            tz = tz * ((self._L1 + self._L2) / d12)
            d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
            d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)

        angle0 = -math.pi / 2 + math.atan2(tz, tx)
        angle1 = math.atan2(ty, d_projXZ) + math.acos(
            (self._L1 * self._L1 + d12 * d12 - self._L2 * self._L2) /
            (2 * self._L1 * d12))
        angle2 = -math.pi + math.acos(
            (self._L1 * self._L1 + self._L2 * self._L2 - d12 * d12) /
            (2 * self._L1 * self._L2))

        return [angle0, angle1, angle2]

    def InverseLegB(self, x, y, z):
        # Leg 2 inverse kinematics.

        rotation = -math.pi * 2 / 3

        rx = math.cos(rotation) * x - math.sin(rotation) * y
        ry = math.sin(rotation) * x + math.cos(rotation) * y

        tx = rx - self._StartX
        ty = ry - self._StartY
        tz = -(z - self._StartZ)

        d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
        d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)
        if (d12 > (self._L1 + self._L2)):
            tx = tx * ((self._L1 + self._L2) / d12)
            ty = ty * ((self._L1 + self._L2) / d12)
            tz = tz * ((self._L1 + self._L2) / d12)
            d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
            d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)

        angle0 = -math.pi / 2 + math.atan2(tz, tx)
        angle1 = math.atan2(ty, d_projXZ) + math.acos(
            (self._L1 * self._L1 + d12 * d12 - self._L2 * self._L2) /
            (2 * self._L1 * d12))
        angle2 = -math.pi + math.acos(
            (self._L1 * self._L1 + self._L2 * self._L2 - d12 * d12) /
            (2 * self._L1 * self._L2))

        return [angle0, angle1, angle2]

    def InverseLegBFromInitialPoint(self, x, y, z):
        # Leg 1 inverse kinematics.

        rotation = -math.pi * 2 / 3

        rx = math.cos(rotation) * x - math.sin(rotation) * y
        ry = math.sin(rotation) * x + math.cos(rotation) * y

        tx = rx
        ty = ry
        tz = self._L0 + self._L1 + self._L2 - z

        d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
        d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)
        if (d12 > (self._L1 + self._L2)):
            tx = tx * ((self._L1 + self._L2) / d12)
            ty = ty * ((self._L1 + self._L2) / d12)
            tz = tz * ((self._L1 + self._L2) / d12)
            d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
            d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)

        angle0 = -math.pi / 2 + math.atan2(tz, tx)
        angle1 = math.atan2(ty, d_projXZ) + math.acos(
            (self._L1 * self._L1 + d12 * d12 - self._L2 * self._L2) /
            (2 * self._L1 * d12))
        angle2 = -math.pi + math.acos(
            (self._L1 * self._L1 + self._L2 * self._L2 - d12 * d12) /
            (2 * self._L1 * self._L2))

        return [angle0, angle1, angle2]

    def InverseLegC(self, x, y, z):
        # Leg 3 inverse kinematics.

        rotation = math.pi * 2 / 3

        rx = math.cos(rotation) * x - math.sin(rotation) * y
        ry = math.sin(rotation) * x + math.cos(rotation) * y

        tx = rx - self._StartX
        ty = ry - self._StartY
        tz = -(z - self._StartZ)

        d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
        d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)
        if (d12 > (self._L1 + self._L2)):
            tx = tx * ((self._L1 + self._L2) / d12)
            ty = ty * ((self._L1 + self._L2) / d12)
            tz = tz * ((self._L1 + self._L2) / d12)
            d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
            d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)

        angle0 = -math.pi / 2 + math.atan2(tz, tx)
        angle1 = math.atan2(ty, d_projXZ) + math.acos(
            (self._L1 * self._L1 + d12 * d12 - self._L2 * self._L2) /
            (2 * self._L1 * d12))
        angle2 = -math.pi + math.acos(
            (self._L1 * self._L1 + self._L2 * self._L2 - d12 * d12) /
            (2 * self._L1 * self._L2))

        return [angle0, angle1, angle2]

    def InverseLegCFromInitialPoint(self, x, y, z):
        # Leg 1 inverse kinematics.

        rotation = math.pi * 2 / 3

        rx = math.cos(rotation) * x - math.sin(rotation) * y
        ry = math.sin(rotation) * x + math.cos(rotation) * y

        tx = rx
        ty = ry
        tz = self._L0 + self._L1 + self._L2 - z

        d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
        d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)
        if (d12 > (self._L1 + self._L2)):
            tx = tx * ((self._L1 + self._L2) / d12)
            ty = ty * ((self._L1 + self._L2) / d12)
            tz = tz * ((self._L1 + self._L2) / d12)
            d_projXZ = math.sqrt(tx * tx + tz * tz) - self._L0
            d12 = math.sqrt(d_projXZ * d_projXZ + ty * ty)

        angle0 = -math.pi / 2 + math.atan2(tz, tx)
        angle1 = math.atan2(ty, d_projXZ) + math.acos(
            (self._L1 * self._L1 + d12 * d12 - self._L2 * self._L2) /
            (2 * self._L1 * d12))
        angle2 = -math.pi + math.acos(
            (self._L1 * self._L1 + self._L2 * self._L2 - d12 * d12) /
            (2 * self._L1 * self._L2))

        return [angle0, angle1, angle2]


if __name__ == '__main__':
    tk = TripedalKinematics()
    print(tk.InverseLegA(0, 0, -180))
    #print(tk.InverseLegB(10,0,-180))
    #print(tk.InverseLegC(10,0,-180))

    print('')
    print(tk.InverseLegAFromInitialPoint(0, 0, 50))
    #print(tk.InverseLegBFromInitialPoint(0,0,50))
    #print(tk.InverseLegCFromInitialPoint(0,0,50))