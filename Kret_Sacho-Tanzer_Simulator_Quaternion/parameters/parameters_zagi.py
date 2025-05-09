"""This file holds the alternate parameters we are useing for our Final Project.
The parameters are based on the Zagi flying wing UAV and are from the Beard and McLain Small Unmanned Aircraft: Theory and Practice book."""

import numpy as np
from tools.rotations import euler_to_quaternion, quaternion_to_euler

######################################################################################
                #   Initial Conditions
######################################################################################
#   Initial conditions for MAV
north0 = 0.  # initial north position
east0 = 0.  # initial east position
down0 = -100.0  # initial down position
u0 = 25.  # initial velocity along body x-axis
v0 = 0.  # initial velocity along body y-axis
w0 = 0.  # initial velocity along body z-axis
phi0 = 0.  # initial roll angle
theta0 = 0.  # initial pitch angle
psi0 = 0.0  # initial yaw angle
p0 = 0  # initial roll rate
q0 = 0  # initial pitch rate
r0 = 0  # initial yaw rate
Va0 = np.sqrt(u0**2+v0**2+w0**2)
#   Quaternion State
e = euler_to_quaternion(phi0, theta0, psi0)
e0 = e.item(0)
e1 = e.item(1)
e2 = e.item(2)
e3 = e.item(3)
