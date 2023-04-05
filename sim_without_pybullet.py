####################################################
##          SORRY FOR THE MESSY CODE.             ##
##  I call the simulations in the last lines      ##
####################################################

import numpy as np
import pinocchio as pin
import control 
import matplotlib.pyplot as plt
from scipy import integrate

def get_M( a1, a2, a3, q2 ):
    return np.array( [ [ a1 + a2 + 2. * a3 * np.cos(q2) , a2 + a3 * np.cos(q2) ], [ a2 + a3 * np.cos(q2), a2 ] ] )

def get_potential_energy( a4, a5, q , g=9.81):
    return ( a4 * np.cos( q[ 0 ] ) + a5 * np.cos( q[ 0 ] + q[ 1 ] ) ) * g 

def get_kinetic_energy( qdot, M ):
    return ( 1 / 2 ) * qdot @ M @ qdot

def get_coriolis( a3, q2, q1dot, q2dot ):
    return np.array( [ [- a3 *  q2dot * np.sin( q2 ), - a3 * ( q1dot + q2dot ) * np.sin( q2 ) ] ,
                       [ a3 * q1dot * np.sin( q2 )  , 0   ] ] ) @ np.array( [ q1dot, q2dot ] )

def get_gravity ( a4, a5, q1, q2, g = 9.81 ):
    return np.array( [  - a4 * np.sin( q1 ) - a5 * np.sin( q1 + q2 ),  - a5 * np.sin( q1 + q2 ) ] ) * g 

def swingUpControl(q, qdot, a1,a2,a3,a4,a5):
     
    kv = 0.4  # choosen to be greater than zero
    kd = 0.15 # found by matlab script
    kp = 2.  # found by matlab script

    M = get_M( a1, a2, a3, q[1] )
    M_inv = np.linalg.inv( M )

    B = np.array( [1, 0] ) # TODO ?? maybe make it column 

    c = get_coriolis( a3, q[1], qdot[0], qdot[1] )
   
    g = get_gravity( a4, a5, q[0], q[1])
    
    pot_energy = get_potential_energy( a4, a5, q )
    
    
    kin_energy = get_kinetic_energy( qdot, M )
    
    E = pot_energy + kin_energy
    
    E_ref = get_potential_energy( a4, a5, [ 0, 0 ])
    
    lam = ( E - E_ref ) + kd *  M_inv[ 0, 0 ] 
   
    tau1 =( - kv * qdot[0] -  kp * q[0] + kd * M_inv[0, :] @ ( c + g ) ) / lam
    tau1 = np.squeeze(tau1).item()
    return np.array([tau1, 0])

m1 = 1.
m2 = 1.
l1 = 0.5 
l2 = 0.5 
d1 = 0.25 
d2 = 0.25 
I_1zz = ( 1. / 12. ) * m1 * np.power( l1, 2 )
I_2zz = ( 1. / 12. ) * m2 * np.power( l2, 2 )
g = -9.81

#a1 = I_1zz + m1 * np.power(d1,2) + m2 * np.power(l1,2) 
#a2 = I_2zz + m2 * np.power(d2,2) 
#a3 = m2 * l1 * d2
#a4 = g * ( m1 * d1 + m2 * l1 )
#a5 = g * ( m2 * d2 )       

a1 = 0.0308
a2 = 0.0106
a3 = 0.0095
a4 = 0.2085 
a5 = 0.0630  
params = [a1, a2,a3,a4,a5]

def int_pendubot_sim(t, x, a1=0.0308, a2=0.0106 ,a3=0.0095 ,a4=0.2085  ,a5=0.0630 , LQR=True):
    
    M = get_M( a1, a2, a3, x[ 1 ] ) # mass matrix

    M_inv = np.linalg.inv( M ) # inverse M

    c = get_coriolis( a3, x[ 1 ], x[ 2 ], x[ 3 ] ) # coriolis vector

    g = get_gravity( a4, a5, x[ 0 ], x[ 1 ] ) # gravity vector

    K = np.array( [ -40.6715, -40.2304, -7.5433, -5.2894 ] )
    # K = np.array(  [-50.4271,  -47.5548 ,  -9.2748  , -6.2219] )

    if( abs( x[0] ) + abs( x[1] ) + 0.1 * abs( x[2] ) + 0.1 * abs( x[3] )  < 0.5  and LQR ):
            tau =  np.array( [ - K @ x, 0 ] )
    else:
            tau = swingUpControl( x[ : 2 ] , x[ 2 : 4 ], a1,a2,a3,a4,a5 )

    x_dot = np.array( [ 0.0, 0.0, 0.0, 0.0 ] )
    x_dot[ : 2 ] = x[ 2 : 4 ]
    x_dot[ 2 : 4 ] = M_inv @ ( tau - c  - g )
    
    return x_dot

def euler_pendulum_sim(x, t, a1=a1, a2=a2 ,a3=a3 ,a4=a4 ,a5=a5 ,LQR =True):

    x_12 = np.array( [ x[ : 2 ] ] ) 
    x_34 = np.array( [ x[ 2 : 4 ] ] )
    dt = t[ 1 ] - t[ 0 ]
    tau_log = [0]

    for i in range( int( t[:-1].size ) ):

        M = get_M( a1, a2, a3, x_12[ -1, 1 ] ) # mass matrix
        M_inv = np.linalg.inv( M ) # inverse M
        c = get_coriolis( a3, x_12[ -1, 1 ], x_34[ -1, 0 ], x_34[ -1, 1 ] ) # coriolis vector
        g = get_gravity( a4, a5, x_12[ -1, 0 ], x_12[ -1, 1 ] ) # gravity vector

        K = np.array( [ -40.6715, -40.2304, -7.5433, -5.2894 ] )

        if( abs( x_12[ -1 , 0 ] ) + abs( x_12[ -1 , 1 ] ) + 0.1 * abs( x_34[-1,0] ) + 0.1 * abs( x_34[-1,1] )  < 0.5  and LQR ):
                tau =  np.array( [ - K @ x, 0 ] )
        else:
                tau =  swingUpControl( x_12[ -1, : ] , x_34[ -1, : ], a1,a2,a3,a4,a5)

        tau_log += [tau[0]] # logging

        x_12_next = x_12[ -1, : ] + dt * x_34[ -1, : ]
        x_34_next = x_34[ -1, : ] + dt * M_inv @ ( tau - c  - g )

        x_12 = np.append( x_12,[ x_12_next ], axis = 0)
        x_34 = np.append( x_34,[ x_34_next ], axis = 0)

    return np.stack( [ x_12, x_34 ], axis =  0 ), tau_log 

def simulate_w_scipy(x_init, sim_duration, simDT):

    t = np.linspace(0, sim_duration, int( sim_duration / simDT ) )

    sol = integrate.solve_ivp( int_pendubot_sim, np.array( [0, sim_duration ] ), x_init, method='RK45' , max_step = simDT)
    plt.plot(sol.t, sol.y[0,:], 'b', label='q1(t)' )
    plt.plot(sol.t ,sol.y[1,:], 'g', label='q2(t)')
    plt.legend(loc='best')
    plt.grid()
    plt.show()

def simulate_w_euler(x_init, sim_duration, simDT):

    t = np.linspace(0, sim_duration, int( sim_duration / simDT ) )
    sol, tau_log = euler_pendulum_sim(np.array(x_init), t)

    plt.plot(t, sol[0,:,0], 'b', label='q1(t)' )
    plt.plot(t ,sol[0,:,1], 'g', label='q2(t)')
    plt.plot(t ,tau_log, 'r', label='tau(t)')
    plt.legend(loc='best')
    plt.grid()
    plt.show()


sim_duration = 2.5
simDT = 1 / 1000

x_init =  np.array( [ - 5. * np.pi / 6., 0., 0., 0. ]  )
#simulate_w_euler(x_init, sim_duration, simDT)
simulate_w_scipy(x_init, sim_duration, simDT)

