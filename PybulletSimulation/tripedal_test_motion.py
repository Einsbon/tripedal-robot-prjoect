import pybullet as p
import pybullet_data
import numpy as np
import math
import os
import csv

from motorController import MotorController

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

p.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=30, cameraPitch=-15, cameraTargetPosition=[-0.05, 0, 0.1])
planeID = p.loadURDF("plane.urdf")
p.changeDynamics(planeID, -1, lateralFriction=0.5)
robotID = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/model/tripedalRobot.urdf', [0, 0, 0.25],
                     p.getQuaternionFromEuler([0, 0, 0]),
                     useFixedBase=False)

motor = MotorController(robotID, physicsClient, fixedTimeStep, motor_kp, motor_kd, motor_torque,
                                   motor_max_velocity)
print(motor.getRevoluteJoint_nameToId())

motor.setMotorsAngleInFixedTimestepSyncRealTime([0, 0, 0, 0, 0, 0, 0, 0, 0], 1, 0)
motor.NoCommandSyncRealTime(3, 1)

f = open(os.path.abspath(os.path.dirname(__file__)) + '/testMotion.CSV', 'r', encoding='utf-8')
rdr = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
for line in rdr:
    print(list(np.multiply(line[0:9], (math.pi / 180))), line[9] / 1000, line[10] / 1000)
    motor.setMotorsAngleInFixedTimestepSyncRealTime(np.multiply(line[0:9], (math.pi / 180)), line[9] / 1000,
                                                               line[10] / 1000, 1.0)

motor.NoCommandSyncRealTime(10, 1)
