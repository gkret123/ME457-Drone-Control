import numpy as np
from Aircraft import Aircraft as ac
from scipy.spatial.transform import Rotation as R
import Parameters_Test as p
from EOMs_main import rigid_body as rb
"""
##### Case 1 ######

elevator = -0.15705144
aileron = 0.01788999
rudder = 0.01084654
throttle = 1.

mav = ac(p)

# quat = [x, y, z, w]
quaternion = [2.47421558e-01, 6.56821468e-02, 2.30936730e-01, 9.38688796e-01]
rotation = R.from_quat(quaternion)
euler = rotation.as_euler('xyz', degrees=False)
"""
state = np.array([[ 6.19506532e+01],
 [ 2.22940203e+01],
 [-1.10837551e+02],
 [ 2.73465947e+01],
 [ 6.19628233e-01],
 [ 1.42257772e+00],
 [euler[0]],
 [euler[1]],
 [euler[2]],
#  [ 9.38688796e-01],
#  [ 2.47421558e-01],
#  [ 6.56821468e-02],
#  [ 2.30936730e-01],
 [ 4.98772167e-03],
 [ 1.68736005e-01],
 [ 1.71797313e-01]]).flatten()
"""

elevator = -0.2
aileron = 0.0
rudder = 0.005
throttle = 0.5
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

state = np.array([[north0, east0, down0, u0, v0, w0, phi0, theta0, psi0, p0, q0, r0]]).flatten()
# T_p, Q_p = mav._motor_thrust_torque(mav._Va, delta.throttle)
# print("Propeller Forces and Torque", "\n")
# print("T_p: " , T_p)
# print("Q_p: " , Q_p, "\n\n")

forcesAndMoments = mav.get_aero_forces(state, [0, 0, 0], [elevator, aileron, rudder])
print("Forces and Moments : Case 2" , "\n")
print("fx: " , forcesAndMoments.item(0))
print("fy: " , forcesAndMoments.item(1))
print("fz: " , forcesAndMoments.item(2))
print("Mx: " , forcesAndMoments.item(3))
print("My: " , forcesAndMoments.item(4))
print("Mz: " , forcesAndMoments.item(5) , "\n\n")

x_dot = mav.x_dot(0, state, forcesAndMoments)
print("State Derivatives : Case 2", "\n")
print("north_dot: ", x_dot.item(0))
print("east_dot: ", x_dot.item(1))
print("down_dot: ", x_dot.item(2))
print("   u_dot: ", x_dot.item(3))
print("   v_dot: ", x_dot.item(4))
print("   w_dot: ", x_dot.item(5))
print("  phi_dot: ", x_dot.item(6))
print("  theta_dot: ", x_dot.item(7))
print("  psi_dot: ", x_dot.item(8))
# print("  e3_dot: ", x_dot.item(9))
print("   p_dot: ", x_dot.item(9))
print("   q_dot: ", x_dot.item(10))
print("    r_dt: ", x_dot.item(11) , "\n\n\n")
"""

"""
# Rigid body test:
quaternion = [0, 0, 0, 1]
rotation = R.from_quat(quaternion)
euler = rotation.as_euler('xyz', degrees=False)

state = np.array([[5], [2], [-20], [5], [0], [0], [euler[0]], [euler[1]], [euler[2]], [1], [0.5], [0]]).flatten()

U = np.array([10, 5, 0, 0, 14, 0])
body = rb(p, gravity=False)


state_dot = body.x_dot(0, state, U)

print(state_dot)
"""