import pybullet as pb
import time
import numpy as np
import pinocchio as pin

import sim_utils
import controller

def simulate():

    simDT = 1/240 # simulation timestep
    simTime = 25 # total simulation time in seconds
    q0 = np.array( [ - 5 * np.pi / 6, 0 ] ) # initial configuration

    robotID, robotModel, params = sim_utils.simulationSetup(simDT)

    a1 = params[0]
    a2 = params[1]
    a3 = params[2]
    a4 = params[3]
    a5 = params[4]
    params = [a1, a2,a3,a4,a5]
   
    # we are going to consider both revolute joints, so we fill the whole
    # joint indices list
    nDof = 2
    jointIndices = range(nDof)
    for i in jointIndices:
        pb.resetJointState(robotID, i, q0[i])

    q, qdot = sim_utils.getState(robotID, jointIndices) 
    #pin.computeAllTerms(robotModel.model, robotModel.data, q, qdot)

    theta = 0.5
    #K = controller.lqr( a1, a2, a3, a4, a5 )
    K = np.array( [ -43.6130,  -43.4783,  -13.3053,   -7.3708 ] ) # for parameters of own robot 
    #print(K)
    #K = controller.lqr_pin(robotModel)

    for i in range(int(simTime/simDT)):
        pin.computeAllTerms(robotModel.model, robotModel.data, q, qdot)
        # read the current joint state from the simulator
        q, qdot = sim_utils.getState(robotID, jointIndices)    

        if( abs( q[0] ) + abs( q[1] ) + 0.1 * abs( qdot[0] ) + 0.1 * abs( qdot[1] )  < theta ):
            tau1 = - K @ np.array( [ q[0], q[1], qdot[0], qdot[1] ] )
            tau =  np.array( [ tau1, 0 ] )
        else:
            tau = controller.swingUpControl(q, qdot, params)
        
        # send the torque command to the simulator
        pb.setJointMotorControlArray(
            robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces = tau)

        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)


    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()

simulate()



