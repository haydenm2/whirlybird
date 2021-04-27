# This python script is used to calculate state derivatives for the Whirlybird
# so that students can verify that their code is calculating correct values

import numpy as np

# Physical parameters for Whirlybird
g = 9.81        # m/s^2
l1 = 0.85       # m
l2 = 0.3048     # m
m1 = 0.891      # kg
m2 = 1.0        # kg
d = 0.178       # m
h = 0.65        # m
r = 0.12        # m
Jx = 0.0047     # kg*m^2
Jy = 0.0014     # kg*m^2
Jz = 0.0041     # kg*m^2
km = 0.1        # N/PWM -- this is not the actual value
                #          you will measure it later

# PWM commands to props
ul = 25.0       # PWM
ur = 17.0       # PWM

# States (inputs to the function that calculates state derivatives)
phi      = 10.0*np.pi/180.0    # rad
phid   = 45.0*np.pi/180.0    # rad/s
theta    = -20.0*np.pi/180.0   # rad
thetad = 15.0*np.pi/180.0    # rad/s
psi      = 45.0*np.pi/180.0    # rad
psid   = -20.0*np.pi/180.0   # rad/s

fl = km*ul
fr = km*ur

cphi     = np.cos(phi)
sphi     = np.sin(phi)
ctheta   = np.cos(theta)
stheta   = np.sin(theta)

# Your code goes here.
# This is the code that you'll have to add to whirlybird_sim.py
################################################
M = np.zeros((3,3))
c = np.zeros((3,1))
dPdq = np.zeros((3,1))
Q = np.zeros((3,1))

M[0,0] = Jx
M[0,1] = 0
M[0,2] = -Jx*stheta
M[1,0] = 0
M[1,1] = m1*l1**2+m2*l2**2+Jy*cphi**2+Jz*sphi**2
M[1,2] = (Jy-Jz)*sphi*cphi*ctheta
M[2,0] = -Jx*stheta
M[2,1] = (Jy-Jz)*sphi*cphi*ctheta
M[2,2] = (m1*l1**2+m2*l2**2+Jy*sphi**2+Jz*cphi**2)*ctheta**2+Jx*stheta**2

c[0,0] = -thetad**2*(Jz-Jy)*sphi*cphi+psid**2*(Jz-Jy)*sphi*cphi*ctheta**2-thetad*psid*ctheta*(Jx-(Jz-Jy)*(cphi**2-sphi**2)) 
c[1,0] = psid**2*stheta*ctheta*(-Jx+m1*l1**2+m2*l2**2+Jy*sphi**2+Jz*cphi**2)-2*phid*thetad*(Jz-Jy)*sphi*cphi-phid*psid*ctheta*(-Jx+(Jz-Jy)*(cphi**2-sphi**2))
c[2,0] = thetad**2*(Jz-Jy)*sphi*cphi*stheta-phid*thetad*ctheta*(Jx+(Jz-Jy)*(cphi**2-sphi**2))-2*phid*psid*(Jz-Jy)*ctheta**2*sphi*cphi+2*thetad*psid*stheta*ctheta*(Jx-m1*l1**2-m2*l2**2-Jy*sphi**2-Jz*cphi**2)

dPdq[0,0] = 0
dPdq[1,0] = (m1*l1-m2*l2)*g*ctheta
dPdq[2,0] = 0

Q[0,0] = d*(fl-fr)
Q[1,0] = l1*(fl+fr)*cphi
Q[2,0] = l1*(fl+fr)*ctheta*sphi+d*(fr-fl)*stheta

b = Q-c-dPdq

xdot = np.linalg.inv(M).dot(b) 
   
phiddot   = xdot[0] 
thetaddot = xdot[1]
psiddot   = xdot[2]

################################################

# Print out the results
print("M=",+M)
print("c=",+c)
print("dPdq=",+dPdq)
print("Q=",+Q)
print("phiddot=",+phiddot)
print("thetaddot=",+thetaddot)
print("psiddot=",+psiddot)

# Results from correct dynamics
#('M=', array([[ 4.70000000e-03,  0.00000000e+00,  1.60749467e-03],
#       [ 0.00000000e+00,  7.38131955e-01, -4.33881637e-04],
#       [ 1.60749467e-03, -4.33881637e-04,  6.54577092e-01]]))
#('c=', array([ 0.00020376, -0.02956825, -0.04448925]))
#('dPdq=', array([0.        , 4.17177996, 0.        ]))
#('Q=', array([0.1424    , 3.51576368, 0.63124167]))
#('phiddot=', 29.926773784266572)
#('thetaddot=', -0.8481305355872574)
#('psiddot=', 0.9582611593110048)
