import pybullet as p
import time
from time import sleep
import pybullet_data
import numpy as np
import math
import os
import csv

import motorController
import tripedal_kinematics
import tripedal_walkGenerator

motor_kp = 0.3
motor_kd = 0.3
motor_torque = 1.5
motor_max_velocity = 20

fixedTimeStep = 1 / 250
numSolverIterations = 200

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.setTimeStep(fixedTimeStep)
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetDebugVisualizerCamera(cameraDistance=0.4,
                             cameraYaw=20,
                             cameraPitch=-25,
                             cameraTargetPosition=[0.3, 0, 0.1])
planeID = p.loadURDF("plane.urdf")

p.changeDynamics(planeID, -1, lateralFriction=0.5)
robotID = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) +
                     '/model/tripedalRobot.urdf', [0, 0, 0.25],
                     p.getQuaternionFromEuler([0, 0, -0.15]),
                     useFixedBase=False)
motorsController = motorController.MotorController(robotID, physicsClient,
                                                   fixedTimeStep, motor_kp,
                                                   motor_kd, motor_torque,
                                                   motor_max_velocity)
kinematicTool = tripedal_kinematics.TripedalKinematics()
print(motorsController.getRevoluteJoint_nameToId())

motorsController.setMotorsAngleInFixedTimestepSyncRealTime(
    [0, 0, 0, 0, 0, 0, 0, 0, 0], 1, 0)
motorsController.NoCommandSyncRealTime(1, 1)

wg = tripedal_walkGenerator.WalkGenerator()
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

motorsController.NoCommandSyncRealTime(5, 1)
motorsController.setMotorsAngleInFixedTimestepSyncRealTime(
    kinematicTool.GetMotorAnglesFromGroundCenterPoints(wg._walkPointA0[0],
                                                       wg._walkPointB0[0],
                                                       wg._walkPointC0[0]), 1,
    0)

#p.resetBasePositionAndOrientation(robotID, [0, 0.03, 0.185], p.getQuaternionFromEuler([0, 0, -0.1]))
wg.ShowGaitPoint3D()
motorsController.NoCommandSyncRealTime(2, 1)

for _ in range(5):
    for i in range(wg._bodyMovePoint):
        motorsController.setMotorsAngleInFixedTimestepSyncRealTime(
            kinematicTool.GetMotorAnglesFromGroundCenterPoints(
                wg._walkPointA0[i], wg._walkPointB0[i], wg._walkPointC0[i]),
            0.06, 0)
    for i in range(wg._legMovePoint):
        motorsController.setMotorsAngleInFixedTimestepSyncRealTime(
            kinematicTool.GetMotorAnglesFromGroundCenterPoints(
                wg._walkPointA1[i], wg._walkPointB1[i], wg._walkPointC1[i]),
            0.06, 0)
    for i in range(wg._bodyMovePoint):
        motorsController.setMotorsAngleInFixedTimestepSyncRealTime(
            kinematicTool.GetMotorAnglesFromGroundCenterPoints(
                wg._walkPointA2[i], wg._walkPointB2[i], wg._walkPointC2[i]),
            0.06, 0)
    for i in range(wg._legMovePoint):
        motorsController.setMotorsAngleInFixedTimestepSyncRealTime(
            kinematicTool.GetMotorAnglesFromGroundCenterPoints(
                wg._walkPointA3[i], wg._walkPointB3[i], wg._walkPointC3[i]),
            0.06, 0)
    for i in range(wg._bodyMovePoint):
        motorsController.setMotorsAngleInFixedTimestepSyncRealTime(
            kinematicTool.GetMotorAnglesFromGroundCenterPoints(
                wg._walkPointA4[i], wg._walkPointB4[i], wg._walkPointC4[i]),
            0.06, 0)
    for i in range(wg._legMovePoint):
        motorsController.setMotorsAngleInFixedTimestepSyncRealTime(
            kinematicTool.GetMotorAnglesFromGroundCenterPoints(
                wg._walkPointA5[i], wg._walkPointB5[i], wg._walkPointC5[i]),
            0.06, 0)

motorsController.NoCommandSyncRealTime(20, 1)
