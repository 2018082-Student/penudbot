# penudbot

Cheers lads

I wrote some code - still not working tho!

Feel free to edit

Basic Idea:
  - run energy based swing up until we are in vicinity of UEP (upward equilibrium point)
  - switch to LQR to stabilize
  

Current Issues:
  - two different parametrizations of the equations of motion: Xin_Liu (the pdf we were provided with) and Lanaris PDF on the pensubot that I found online
  - the LQR is working with Lanaris parametrization
  - the energy swing up is not working with both parametrization ( maybe my gain parametrization is off ? ) 
