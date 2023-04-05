import numpy as np
from autograd import jacobian
import pinocchio as pin
import control 

def get_M( a1, a2, a3, q ):
    q2 = q[1]
    return np.matrix( [ [ a1 + a2 + 2 * a3 * np.cos(q2) , a2 + a3 * np.cos(q2) ], [ a2 + a3 * np.cos(q2), a2 ] ] )

def get_potential_energy( a4, a5, q ):
    return a4 * np.cos( q[ 0 ] ) + a5 * np.cos( q[ 0 ] + q[ 1 ] )

def get_kinetic_energy( qdot, M ):
    return ( 1 / 2 ) * qdot @ M @ qdot

# Coriolis Vector 
def get_coriolis( a3, q2, q1dot, q2dot ):
    return np.array( [ [- a3 *  q2dot * np.sin( q2 ), - a3 * ( q1dot + q2dot ) * np.sin( q2 ) ] ,
                      [ a3 * q1dot * np.sin( q2 ) , 0 ] ] ) @ np.array( [ q1dot, q2dot ] )

# Gravity Vector
def get_gravity ( a4, a5, q1, q2 ):
    return np.array( [ a4 * np.cos( q1 ) + a5 * np.cos( q1 + q2 ), a5 * np.cos( q1 + q2 ) ] )

# swing up controller with the model computation done in pinocchio 
def SwingUpControl_Pin(robotModel, q, qdot):

    pin.computeAllTerms(robotModel.model, robotModel.data, q, qdot)

    kv = 0.4
    kd = 0.65
    kp = 0.3

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
      
    tau1 = ( - kv * q1dot -  kp * q1 + kd * B.T @ Minv @ ( H + G ) ) / lam

    return np.array([tau1.flatten()[0], 0])

def SwingUpControl(q, qdot, params):
    a1 = params[0]
    a2 = params[1]
    a3 = params[2]
    a4 = params[3]
    a5 = params[4]
     
    kv = 0.4  # choosen to be greater than zero
    kd = 0.65 # found by matlab script
    kp = 0.3  # found by matlab script

    M = get_M(a1, a2, a3, q)
    M_inv = np.linalg.inv( M )

    B = np.array( [1, 0] ) # TODO ?? mybe make it column 

    c = get_coriolis( a3, q[1], qdot[0], qdot[1] )

    g = get_gravity( a4, a5, q[0], q[1])
    
    pot_energy = get_potential_energy( a4, a5, q )
    
    kin_energy = get_kinetic_energy( qdot, M )
    
    E = pot_energy + kin_energy
    
    E_ref = get_potential_energy( a4, a5, [ 0, 0 ])
    
    lam = ( E - E_ref ) + kd * ( B.T @ M_inv @ B )
      
    tau1 =( - kv * qdot[0] -  kp * q[0] + kd * B.T @ M_inv @ ( c + g ) ) / lam
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

# linearizing 
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

# linearizing  
def get_B(a1, a2, a3, a4, a5):
    delta = a1 * a2 - np.power( a3, 2 )
    upper_rows = np.zeros( (2, 1) )
    b1 = 1
    b2 = 0
    lower_rows = np.array( [ [ ( b1 * a2 - b2 * ( a2 * a3 ) ) / delta , 
                              ( - b1 * ( a2 + a3 ) + b2 * ( a1 + a2 + 2 * a3 ) ) / delta ] ] ).T
    B = np.concatenate((upper_rows, lower_rows), axis=0)
    return B

def lqr(a1, a2 , a3, a4, a5):

    A = get_A(a1,a2,a3,a4,a5)
    B = get_B(a1,a2,a3,a4,a5)
    Q = np.diag( [10, 10, 1, 1] )
    R = np.diag( [1] )
    K, S, E = control.lqr(A, B, Q, R)


    return K



################################################################################
### Functions implemented according to the PENDUBOT PDF by Lanari and Oriolo ###
################################################################################

def get_M_lanari( a1, a2, a3, q ):
    q2 = q[1]
    return np.matrix( [ [ a1 + 2 * a2 * np.cos(q2) , a3 + a2 * np.cos(q2) ], [ a3 + a2 * np.cos(q2), a3 ] ] )

def get_A_lanari(a1, a2, a3, a4, a5, f1, f2):
    # A = [    0          I    ]
    #     [-M^-1 * H  -M^-1 * F]
    # H = dg/dq | q = q_e

    zeros = np.zeros( (2, 2) )
    eye = np.diag( [ 1, 1 ])
    upper_rows = np.concatenate((zeros, eye), axis=1)

    M = get_M_lanari(a1, a2, a3, [ 0, 0 ])
    F = np.diag( [ f1, f2 ])
    #lin_G = get_linearized_G_lanari( a4, a5, [ 0, 0 ] ) #linearized gravity vector evaluated at q = (0,0)
    M_inv = np.linalg.inv( M )
    #lower_rows = np.concatenate((- M_inv @ lin_G , - M_inv @ F ), axis=1)
    return 0 
    #return np.concatenate((upper_rows, lower_rows), axis=0)   

# Gravity from Lanari
def get_g_lanari(a4, a5 ,q1, q2):
    return np.array( [ a4 * np.sin( q1 ) + a5 * np.sin( q1 + q2 ) , a5 * np.sin( q1 + q2 ) ])

# Coriolis from Lanari
def get_coriolis_lanari( a2, q2, q1dot, q2dot ):
    return np.array( [ a2 * np.sin( q2 ) * q2dot * ( q2dot + 2 * q1dot ) , a2 * np.sin( q2 ) * (q2dot * q2dot) ])

def get_B_lanari(a1, a2, a3, a4, a5):
    upper_rows = np.zeros( (2, 1) )

    M = get_M_lanari(a1, a2, a3, [ 0, 0 ])
    M_inv = np.linalg.inv( M )
    lower_rows = np.array( M_inv @ np.array( [ 1 , 0 ] ) ).T
    B = np.concatenate((upper_rows, lower_rows), axis=0)
    return B
