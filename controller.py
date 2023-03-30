import numpy as np
from autograd import jacobian
import pinocchio as pin
import control 

def get_M(a1,a2,a3, q):
    q2 = q[1]
    return np.matrix( [ [ a1 + a2 + 2 * a3 * np.cos(q2) , a2 + a3 * np.cos(q2) ], [ a2 + a3 * np.cos(q2), a2 ] ] )

def get_M_lanari(a1, a2, a3, q):
    q2 = q[1]
    return np.matrix( [ [ a1 + 2 * a2 * np.cos(q2) , a3 + a2 * np.cos(q2) ], [ a3 + a2 * np.cos(q2), a3 ] ] )

def get_potential_energy(a4,a5, q):
    return a4 * np.cos( q[ 0 ] ) + a5 * np.cos( q[ 0 ] + q[ 1 ] )

def get_kinetic_energy(qdot, M ):
    return ( 1 / 2 ) * qdot @ M @ qdot

# Coriolis 
def get_H(a3, q, qdot):
    q1dot = qdot[0]
    q2dot = qdot[1]
    q2 = q[1]
    return a3 * np.array( [ - 2 * q1dot * q2dot - np.power( q2dot, 2 ) , np.power( q1dot, 2) ] ) * np.sin( q2 )

def get_G (a4, a5, q1, q2 ):
    return np.array( [ - a4 * np.sin( q1 ) - a5 * np.sin( q1 + q2 ), - a5 * np.sin( q1 + q2 ) ] )

def get_G_jacobian(a4, a5, q1, q2):
    return np.array( [ [ - a4 * np.cos( q1 ) - a5 * np.cos( q1 + q2 ), - a5 * np.cos( q1 + q2 )], 
                       [ - a5 * np.cos( q1 + q2 ), - a5 * np.cos( q1 + q2 ) ] ] )

def get_linearized_G_lanari(a4, a5, q):
    q1 = q[0]
    q2 = q[1]
    return np.array( [ [ a4 * np.cos( q1 ) + a5 * np.cos( q1 + q2 ) , a5 * np.cos( q1 + q2 ) ] , 
                       [ a5 * np.cos( q1 + q2 ) , a5 * np.cos( q1 + q2 ) ] ] )

def SwingUpControl_Pin(robotModel, q, qdot):

    pin.computeAllTerms(robotModel.model, robotModel.data, q, qdot)

    kv = 10
    kd = 9
    kp = 25

    q1dot = qdot[0] 
    q1 = q[0]

    Minv = pin.computeMinverse( robotModel.model, robotModel.data, q )
    B = np.array([[1, 0]]).T
    H = pin.computeCoriolisMatrix( robotModel.model, robotModel.data, q, qdot ) @ qdot
    G = pin.computeGeneralizedGravity( robotModel.model, robotModel.data, q )
    pot_E = pin.computePotentialEnergy( robotModel.model, robotModel.data, q)
    kin_E = pin.computeKineticEnergy( robotModel.model, robotModel.data, q , qdot)
    E = pot_E + kin_E
    E_ref = pin.computePotentialEnergy( robotModel.model, robotModel.data, np.array([0, 0]) )
    lam = ( E - E_ref ) + kd * ( B.T @ Minv @ B )
      
    tau1 =( - kv * q1dot -  kp * q1 + kd * B.T @ Minv @ ( H + G ) ) / lam

    return np.array([tau1.flatten()[0], 0])

def SwingUpControl(q, qdot, params):
    a1 = params[0]
    a2 = params[1]
    a3 = params[2]
    a4 = params[3]
    a5 = params[4]
    f1 = params[5]
    f2 = params[6]
    q1 = q[0]
    q2 = q[1]
    q1dot = qdot[0]
    q2dot = qdot[1]
     
    kv = 1
    kd = 50
    kp = 1

    M = get_M(a1, a2, a3, q)
    M_inv = np.linalg.inv( M )
    B = np.array( [1, 0] ) # TODO ?? mybe make it column 
    H = get_H( a3, q, qdot )
    G = get_G( a4, a5, q[0], q[1] )
    F = np.diag( [f1, f2])
    pot_energy = get_potential_energy( a4, a5, q )
    kin_energy = get_kinetic_energy( qdot, M )
    E = pot_energy + kin_energy
    E_ref = get_potential_energy(a4,a5, [0,0])
    omega = np.sqrt( np.power( a4 ,2 ) + np.power( a5, 2 ) + 2 * a4 * a5 * np.cos( q2 ) )
    lam = ( E - E_ref ) + kd * ( B.T @ M_inv @ B )      
    tau1 =( - kv * q1dot -  kp * q1 + kd * B.T @ M_inv @ ( H + G ) ) / lam
    tau1 = np.squeeze(tau1).item()
    return np.array([tau1, 0])


def JointPositionControl(robotModel, q, qdot, qdes):
    """
    Joint position controller implemented via inverse dynamics + PD
    """

    KP = np.diag([50,50])
    KD = np.diag([10,10])

    qddot_des = KP@(qdes-q) + KD@(-qdot)    
    torque = pin.rnea(robotModel.model, robotModel.data, q, qdot, qddot_des)

    return torque

def get_A(a1, a2, a3, a4, a5):
    zeros = np.zeros( (2, 2) )
    eye = np.diag( [ 1, 1 ])
    delta = a1 * a2 - np.power( a3, 2 )
    upper_rows = np.concatenate((zeros, eye), axis=1)
    lower_left = np.array( [ [ (a2 * a4 - a3 * a5 ) / delta ,  ( -a3 * a5 ) / delta] , 
                            [ ( - ( a2 + a3 ) * a4 + ( a1 + a3 ) * a5 ) / delta , ( ( a1 + a3 ) * a5 ) / delta ] ] )
    lower_right = zeros = np.zeros( (2, 2) ) 
    lower_rows = np.concatenate((lower_left, lower_right), axis=1)
    return np.concatenate((upper_rows, lower_rows), axis=0)   

def get_A_lanari(a1, a2, a3, a4, a5, f1, f2):
    # A = [    0          I    ]
    #     [-M^-1 * H  -M^-1 * F]
    # H = dg/dq | q = q_e

    zeros = np.zeros( (2, 2) )
    eye = np.diag( [ 1, 1 ])
    upper_rows = np.concatenate((zeros, eye), axis=1)

    M = get_M_lanari(a1, a2, a3, [ 0, 0 ])
    F = np.diag( [ f1, f2 ])
    lin_G = get_linearized_G_lanari( a4, a5, [ 0, 0 ] ) #linearized gravity vector evaluated at q = (0,0)
    M_inv = np.linalg.inv( M )
    lower_rows = np.concatenate((- M_inv @ lin_G , - M_inv @ F ), axis=1)

    return np.concatenate((upper_rows, lower_rows), axis=0)   

def get_B(a1, a2, a3, a4, a5):
    delta = a1 * a2 - np.power( a3, 2 )
    upper_rows = np.zeros( (2, 1) )
    lower_rows = np.array( [ [ ( a4 * a2 - a5 * ( a2 * a3 ) ) / delta , 
                              ( - a4 * ( a2 + a3 ) + a5 * ( a1 + a2 + 2 * a3 ) ) / delta ] ] ).T
    B = np.concatenate((upper_rows, lower_rows), axis=0)
    return B

def get_B_lanari(a1, a2, a3, a4, a5):
    upper_rows = np.zeros( (2, 1) )

    M = get_M_lanari(a1, a2, a3, [ 0, 0 ])
    M_inv = np.linalg.inv( M )
    lower_rows = np.array( M_inv @ np.array( [ 1 , 0 ] ) ).T
    B = np.concatenate((upper_rows, lower_rows), axis=0)
    return B

def lqr_lanari(a1, a2 , a3, a4, a5, f1, f2):

    A = get_A_lanari(a1,a2,a3,a4,a5, f1,f2)
    #print(A)
    B = get_B_lanari(a1,a2,a3,a4,a5)
    Q = np.diag( [10, 10, 0, 0] )
    R = np.diag( [1] )
    
    K, S, E = control.lqr(A, B, Q, R)


    return K

def lqr(a1, a2 , a3, a4, a5, f1, f2):

    A = get_A(a1,a2,a3,a4,a5)
    B = get_B(a1,a2,a3,a4,a5)
    Q = np.diag( [10, 10, 0, 0] )
    R = np.diag( [1] )
    K, S, E = control.lqr(A, B, Q, R)


    return K
