import pybullet as p
import time
import pybullet_data
import math
import os


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(1 / 1000)
#p.setGravity(0, 0, -9.8)
p.setPhysicsEngineParameter(numSolverIterations=1000)
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=20, cameraPitch=0, cameraTargetPosition=[0, 0, 0.1], physicsClientId=physicsClient)

robotPos = [0, 0, 0.25]
robotScale = 1
planeID = p.loadURDF('plane.urdf')
robot = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/model/tripedalRobot.urdf',
                   robotPos,
                   p.getQuaternionFromEuler([0, 0, 0]),
                   useFixedBase=True,
                   globalScaling=robotScale)
p.changeDynamics(robot, -1, linearDamping=0, angularDamping=0)

jointIds = []
paramIds = []


for j in range(p.getNumJoints(robot)):
  p.changeDynamics(robot, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(robot, j)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -math.pi, math.pi, 0))

p.setRealTimeSimulation(1)
while (1):
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = p.readUserDebugParameter(c)
    p.setJointMotorControl2(robot, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)