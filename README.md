# penudbot

Basic Idea:
  - run energy based swing up until we are in vicinity of UEP (upward equilibrium point)
  - switch to LQR to stabilize ( the LQR is working with Lanaris parametrization ) 
  

Current Issues:
  - Swing Up is not working, even unstable when starting from vertain initial conditions (like q0 = [ pi/2, 0 ] )

possible error sources: 
  - gains
  - modelling of urdf ( currently 2nd joint is modeled as active, but with always 0 torque input )
  - dumb typo in code 
