import numpy as np
from Aircraft import Aircraft as ac
from scipy.spatial.transform import Rotation as R
import Parameters_Test as p

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