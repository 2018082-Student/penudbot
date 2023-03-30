import pybullet as p
import time
import pybullet_data
import os
import sys

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

# set camera angle and position
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=-15, cameraTargetPosition=[0,0,0.5])
# disable all additional GUI
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0.5]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("pendulum.urdf",
                     startPos, 
                     startOrientation,
                     useFixedBase = 1)

p.setJointMotorControlArray(robotId, [0,1], p.VELOCITY_CONTROL, forces=[0,0]) #setting robot into torque control mode

p.setJointMotorControlArray(
                bodyIndex=robotId,
                jointIndices=[0,1],
                controlMode=p.TORQUE_CONTROL,
                forces=[0.1,0.1])

print(p.getBasePositionAndOrientation(robotId))                     
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (1000):
    p.stepSimulation()
    #print(p.getBasePositionAndOrientation(boxId))
    
    #p.setJointMotorControlArray(
    #            bodyIndex=robotId,
    #            jointIndices=[0,1],
    #            controlMode=p.TORQUE_CONTROL,
    #            forces=[0.1,0.1])

    time.sleep(1./240.)
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()

