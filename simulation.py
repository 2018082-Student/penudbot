import pybullet as pb
import time
import numpy as np
import pinocchio as pin

import sim_utils
import controller

def simulate():

    simDT = 1/240 # simulation timestep
    simTime = 25 # total simulation time in seconds
    q0 = np.array([np.pi/20, 0]) # initial configuration

    robotID, robotModel, params = sim_utils.simulationSetup(simDT)

    a1 = params[0]
    a2 = params[1]
    a3 = params[2]
    a4 = params[3]
    a5 = params[4]
    f1 = params[5]
    f2 = params[6]
    print(params)
    nDof = 2

    # we are going to consider both revolute joints, so we fill the whole
    # joint indices list
    jointIndices = range(nDof)

    for i in jointIndices:
        pb.resetJointState(robotID, i, q0[i])

    q, qdot = sim_utils.getState(robotID, jointIndices) 

    # in general we need to call this to compute all the kinematic and 
    # dynamic quantities (see pinocchio docs) which can be retrieved 
    # either as members of the robotModel.data object, or via specific
    # functions
    # pin.computeAllTerms(robotModel.model, robotModel.data, q, qdot)

    # set a desired joint configuration
    #qdes = np.array([0,0])
    theta = 0.5
    q_e = np.array( [0,0])
    qdot_e = np.array( [0,0])

    K = controller.lqr_lanari(a1, a2, a3, a4, a5, f1, f2)
    #K = controller.lqr_pin(robotModel)
    #print(K)
    #input("press ENTER to START the simulation:")

    for i in range(int(simTime/simDT)):

        # read the current joint state from the simulator
        q, qdot = sim_utils.getState(robotID, jointIndices)    

        if( abs( q[0] ) + abs( q[1] ) + 0.1 * abs( qdot[0] ) + 0.1 * abs( qdot[1] )  < theta ):
            tau1 = - K @ np.array( [ q[0], q[1], qdot[0], qdot[1] ] )
            tau =  np.array( [tau1[0], 0])
        else:
            #tau = controller.SwingUpControl(q, qdot, params)
            tau = controller.SwingUpControl_Pin(robotModel, q, qdot)
        
        #tau = np.array([0,0])
        # send the torque command to the simulator
        pb.setJointMotorControlArray(
            robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces = tau)

        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)


    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()

simulate()



