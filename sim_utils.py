
import pybullet as pb
import pybullet_data

import pinocchio as pin

import numpy as np

import os
from dataclasses import dataclass

from urdfpy import URDF
#def omega(a4, a5, q2):
 #   return np.sqrt( a4^2 + a5^2 + 2 * a4 * a5 * np.cos(q2))

def find_min_gains(a1, a2 , a3, a4, a5):
    kd = 0
    kp = 0
    kv = 0
    E_ref = 1 # TODO
    q2 = range(0, np.pi, 1000)
    


    return kd, kp, kv

#uses the parameters from Xin and Liu 
def load_model_from_URDF():

    robot = URDF.load('pendubot.urdf')

    m1 = robot.links[1].inertial.mass
    m2 = robot.links[2].inertial.mass
    l1 = 0.5 #robot.links[1].Visual.geometry.length
    l2 = 0.5 # robot.links[2].Geometry.length
    d1 = 0.25 # robot.links[1].Geometry.length / 2
    d2 = 0.25 # robot.links[2].Geometry.length / 2
    I_1zz = ( 1 / 12 ) * m1 * np.power( l1, 2 )
    I_2zz = ( 1 / 12 ) * m2 * np.power( l2, 2 )
    g = 9.81
    f1 = robot.joints[0].dynamics.damping
    f2 = robot.joints[1].dynamics.damping
    
    a1 = I_1zz + m1 * np.power(d1,2) + m2 * np.power(l1,2) 
    a2 = I_2zz + m2 * np.power(d2,2) 
    a3 = m2 * l1 * d2
    a4 = g * ( m1 * d1 + m2 * l1 )
    a5 = g * ( m2 * d2 )

    #Er = (l1+d2)*m2*g+1*m1*g;
    #q2 = 0
    #omega = np.sqrt( a4^2 + a5^2 + 2 * a4 * a5 * np.cos(q2))
    #kd_min = ( 1 / a2 ) *  ( Er + omega ) * ( a1 * a2 - a3^2 * np.power(np.cos(q2), 2) )
    #kv_min = 0 #TODO
    #kp_min = 0 #TODO

    return [a1, a2,a3,a4,a5,f1,f2]

#uses the parameters from the Penudbot Script of Lanari
def load_model_from_URDF_lanari():
    robot = URDF.load('pendubot.urdf')

    m1 = robot.links[1].inertial.mass
    m2 = robot.links[2].inertial.mass
    l1 = 0.5 #robot.links[1].Visual.geometry.length
    l2 = 0.5 # robot.links[2].Geometry.length
    d1 = 0.25 # robot.links[1].Geometry.length / 2
    d2 = 0.25 # robot.links[2].Geometry.length / 2
    I_1zz = ( 1 / 12 ) * m1 * np.power( l1, 2 )
    I_2zz = ( 1 / 12 ) * m2 * np.power( l2, 2 )
    g = -9.81
    f1 = robot.joints[0].dynamics.damping
    f2 = robot.joints[1].dynamics.damping

    a1 = I_1zz + m1 * np.power(d1,2) + I_2zz + m2 * ( np.power(l1,2) + np.power(l2,2) )
    a2 = m2 * l1 * d2
    a3 = I_2zz + m2 * np.power(d2,2)
    a4 = g * ( m1 * d1 +m2 * l1 )
    a5 = g * ( m2 * d2 )

    #Er = (l1+d2)*m2*g+1*m1*g;
    #q2 = 0
    #omega = np.sqrt( a4^2 + a5^2 + 2 * a4 * a5 * np.cos(q2))
    #kd_min = ( 1 / a2 ) *  ( Er + omega ) * ( a1 * a2 - a3^2 * np.power(np.cos(q2), 2) )
    #kv_min = 0 #TODO
    #kp_min = 0 #TODO

    return [a1, a2,a3,a4,a5,f1,f2]

def simulationSetup(simDT):

    physicsClient = pb.connect(pb.GUI) 
    # physicsClient = pb.connect(pb.DIRECT) # for non-graphical version
    pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 

    # set camera angle and position
    pb.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-15, cameraTargetPosition=[0,0,1])
    # disable all additional GUI
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI,0)

    # set gravity and simulation timestep
    pb.setGravity(0,0,-9.81)
    pb.setTimeStep(simDT)
    pb.setRealTimeSimulation(False)

    # load the ground model
    planeId = pb.loadURDF("plane.urdf")

    # load the robot model setting initial position and orientation of the base link
    startPos = [0,0,1]
    startOrientation = pb.getQuaternionFromEuler([0,0,0])

    urdf_filename = "pendubot.urdf"

    robotID = pb.loadURDF(urdf_filename, startPos, startOrientation, useFixedBase=1) # note that here we fix the base link
    
    nDof = pb.getNumJoints(robotID)

    # since pybullet defaults to velocity control set to 0 for all joints, to do
    # force control or to make joints passive we need to deactivate it by setting the maximum force to zero
    pb.setJointMotorControlArray(
        robotID, 
        jointIndices=range(nDof),
        controlMode = pb.VELOCITY_CONTROL,
        forces = [0]*nDof)

    # step simulation one time for initialization
    pb.stepSimulation()

    # build the pinocchio model 
    pin_model = pin.buildModelFromUrdf(urdf_filename)
    pin_data = pin_model.createData()
    robotModel = RobotModel(pin_model, pin_data)
    
    return robotID, robotModel, load_model_from_URDF_lanari()




def getState(robotID, jointList):

    currentJointStates = pb.getJointStates(robotID, jointList)
    q = np.array([currentJointStates[jointIdx][0] for jointIdx in jointList])
    qdot = np.array([currentJointStates[jointIdx][1] for jointIdx in jointList])

    return q, qdot


@dataclass
class RobotModel():
    model: ...
    data: ...
