import pybullet as p
import pybullet_data
import os

from motorController import MotorController
from tripedal_kinematics import TripedalKinematics
from tripedal_walkGenerator import WalkGenerator

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

p.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=20, cameraPitch=-25, cameraTargetPosition=[0.3, 0, 0.1])
planeID = p.loadURDF("plane.urdf")

p.changeDynamics(planeID, -1, lateralFriction=0.5)
robotID = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/model/tripedalRobot.urdf', [0, 0, 0.25],
                     p.getQuaternionFromEuler([0, 0, 0]),
                     useFixedBase=False)
motor = MotorController(robotID, physicsClient, fixedTimeStep, motor_kp, motor_kd, motor_torque,
                                   motor_max_velocity)
kinematicTool = TripedalKinematics()
print(motor.getRevoluteJoint_nameToId())

motor.setMotorsAngleInFixedTimestepSyncRealTime([0, 0, 0, 0, 0, 0, 0, 0, 0], 1, 0)
motor.NoCommandSyncRealTime(1, 1)

wg = WalkGenerator()
wg.SetWalkParameter(moveDirection=0,
                    bodyMovePointsCount=10,
                    legMovePointsCount=8,
                    stepLength=50,
                    stepHeight=0.1,
                    legXYDistanceFromCenter=80,
                    sit=60,
                    swayShift=0.6,
                    swayRadiusMin=24,
                    swayRadiusMax=28,
                    liftPush=0.5,
                    landPull=0.5,
                    damping=0,
                    incline=0)

wg.InitProperty()

motor.NoCommandSyncRealTime(1, 1)

pointA, pointB, pointC, _ = wg.MakeNextPoint()
motor.setMotorsAngleInFixedTimestepSyncRealTime(
    kinematicTool.GetMotorAnglesFromRelativePoints(pointA, pointB, pointC), 1, 0)

motor.NoCommandSyncRealTime(2, 1)

stepLengthControl = p.addUserDebugParameter("Step Length", -70, 70, wg._l)
moveDirectionControl = p.addUserDebugParameter("Move Direction", -3.14, 3.14, wg._moveDirection)

while True:
    try:
        wg._l = p.readUserDebugParameter(stepLengthControl)
    except:
        pass
    try:
        wg._moveDirection = p.readUserDebugParameter(moveDirectionControl)
    except:
        pass

    pointA, pointB, pointC, _ = wg.MakeNextPoint()
    motor.setMotorsAngleInFixedTimestepSyncRealTime(
        kinematicTool.GetMotorAnglesFromRelativePoints(pointA, pointB, pointC), 0.055, 0)
