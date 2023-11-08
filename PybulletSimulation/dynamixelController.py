import numpy as np
import math
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library

BAUDRATE = 1000000

ADDR_DRIVE_MODE = 10
ADDR_OPPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_PWM = 124
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
# Refer to the Minimum Position Limit of product eManual
DXL_MINIMUM_POSITION_VALUE = 0
# Refer to the Maximum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE = 4095


LEN_GOAL_POSITION = 4
LEN_GOAL_ACCELERATION = 4
LEN_GOAL_VELOCITY = 4


PROTOCOL_VERSION = 2.0

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold


class DynamixelController:
    def __init__(self, deviceName: str, baudrate: int, motorIds: list) -> None:
        self.__deviceName = deviceName
        self.__baudrate = baudrate
        self.__motorIds = motorIds

        # motorIds 에 중복된 값이 있는지, int형만 있는지 검사.
        for item in self.__motorIds:
            if not (type(item) is int):
                raise Exception('element of motorIds must be int type')
            if self.__motorIds.count(item) != 1:
                raise Exception('motorIds should not contains same elements')

        self.__portHandler = PortHandler(self.__deviceName)
        self.__packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.ConnectDevice()
        self.CommandMotorDriveMode4()
        self.InitGroupSyncWrite()

    def InitGroupSyncWrite(self):
        self.__groupSyncWritePos = GroupSyncWrite(
            self.__portHandler, self.__packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        self.__groupSyncWriteAccel = GroupSyncWrite(
            self.__portHandler, self.__packetHandler, ADDR_PROFILE_ACCELERATION, LEN_GOAL_ACCELERATION)
        self.__groupSyncWriteVelocity = GroupSyncWrite(
            self.__portHandler, self.__packetHandler, ADDR_PROFILE_VELOCITY, LEN_GOAL_VELOCITY)

    def CommandMotorPos(self, targetPosList: list) -> None:
        for i in range(len(self.__motorIds)):
            if(targetPosList[i] != None):
                radianPos = int(2048 + 2048 * targetPosList[i] / math.pi)
                posByte = [DXL_LOBYTE(DXL_LOWORD(radianPos)), DXL_HIBYTE(DXL_LOWORD(radianPos)), DXL_LOBYTE(
                    DXL_HIWORD(radianPos)), DXL_HIBYTE(DXL_HIWORD(radianPos))]
                self.__groupSyncWritePos.addParam(self.__motorIds[i], posByte)

        self.__groupSyncWritePos.txPacket()
        self.__groupSyncWritePos.clearParam()

    def CommandMotorPosAndTime(self, targetPosList: list, goalTime: float,
                               goalAccTime: float) -> None:
        self.t_CommandMotorTime(goalTime, goalAccTime)
        self.t_CommandMotorPos(targetPosList)

        self.__groupSyncWritePos.txPacket()
        self.__groupSyncWritePos.clearParam()

    def CommandMotorTime(self, goalTime: float, goalAccTime: float) -> None:
        if (goalAccTime != None):
            accMillisec = int(goalAccTime * 1000)
            accByte = [DXL_LOBYTE(DXL_LOWORD(accMillisec)), DXL_HIBYTE(DXL_LOWORD(accMillisec)), DXL_LOBYTE(
                DXL_HIWORD(accMillisec)), DXL_HIBYTE(DXL_HIWORD(accMillisec))]
            for i in range(len(self.__motorIds)):
                self.__groupSyncWriteAccel.addParam(
                    self.__motorIds[i], accByte)

        if (goalTime != None):
            timeMillisec = int(goalTime * 1000)
            timeByte = [DXL_LOBYTE(DXL_LOWORD(timeMillisec)), DXL_HIBYTE(DXL_LOWORD(timeMillisec)), DXL_LOBYTE(
                DXL_HIWORD(timeMillisec)), DXL_HIBYTE(DXL_HIWORD(timeMillisec))]
            for i in range(len(self.__motorIds)):
                self.__groupSyncWriteVelocity.addParam(
                    self.__motorIds[i], timeByte)

        self.__groupSyncWriteAccel.txPacket()
        self.__groupSyncWriteVelocity.txPacket()
        self.__groupSyncWriteAccel.clearParam()
        self.__groupSyncWriteVelocity.clearParam()

    def ConnectDevice(self):
        self.__portHandler.openPort()
        self.__portHandler.setBaudRate(self.__baudrate)

    def DisconnectDevice(self):
        self.__portHandler.closePort()

    def CommandMotorDriveMode4(self):
        for i in self.__motorIds:
            self.__packetHandler.write1ByteTxRx(
                self.__portHandler, i, ADDR_DRIVE_MODE, 4)

    def CommandMotorOperatingMode3(self):
        for i in self.__motorIds:
            self.__packetHandler.write1ByteTxRx(
                self.__portHandler, i, ADDR_OPPERATING_MODE, 3)

    def CommandMotorTorque(self, isTorqueOn: bool):
        if isTorqueOn:
            for i in self.__motorIds:
                self.__packetHandler.write1ByteTxRx(self.__portHandler, i, ADDR_TORQUE_ENABLE,
                                                    TORQUE_ENABLE)
        else:
            for i in self.__motorIds:
                self.__packetHandler.write1ByteTxRx(self.__portHandler, i, ADDR_TORQUE_ENABLE,
                                                    TORQUE_DISABLE)

    def CommandOneMotorPos(self, motorIndex: int, targetPos: float) -> None:
        radianToPosition = int(2048 + 2048 * targetPos / math.pi)
        self.__packetHandler.write4ByteTxOnly(self.__portHandler, self.__motorIds[motorIndex],
                                              ADDR_GOAL_POSITION, radianToPosition)

    def ReadMotorPos(self) -> list:
        posList = []
        for i in self.__motorIds:
            pos, _, _ = self.__packetHandler.read4ByteTxRx(self.__portHandler, i,
                                                           ADDR_PRESENT_POSITION)
            posRadian = (pos - 2048)*math.pi/2048.0
            posList.append(posRadian)
        return posList

    def ReadMotorPwm(self) -> list:
        pwmList = []
        for i in self.__motorIds:
            pwm, _, _ = self.__packetHandler.read2ByteTxRx(
                self.__portHandler, i, ADDR_PRESENT_PWM)
            pwmList.append(pwm)
        return pwmList

    def ReadMotorCurrent(self) -> list:
        currentList = []
        for i in self.__motorIds:
            current, _, _ = self.__packetHandler.read2ByteTxRx(self.__portHandler, i,
                                                            ADDR_PRESENT_CURRENT)
            currentList.append(current)
        return currentList
