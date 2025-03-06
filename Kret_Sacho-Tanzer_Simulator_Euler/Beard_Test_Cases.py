import numpy as np
from Aircraft import Aircraft as ac
from scipy.spatial.transform import Rotation as R
import Parameters_Test as p

def quaternion_to_euler(quaternion):  # from Beard
    """
    converts a quaternion attitude to an euler angle attitude
    :param quaternion: the quaternion to be converted to euler angles in a np.matrix
    :return: the euler angle equivalent (phi, theta, psi) in a np.array
    """
    e0 = quaternion.item(0)
    e1 = quaternion.item(1)
    e2 = quaternion.item(2)
    e3 = quaternion.item(3)
    phi = np.arctan2(2.0 * (e0 * e1 + e2 * e3), e0**2.0 + e3**2.0 - e1**2.0 - e2**2.0)
    # theta = np.arcsin(2.0 * (e0 * e2 - e1 * e3))
    theta = -np.pi/2.0 + np.arctan2(np.sqrt(1+2.0*(e0*e2-e1*e3)), np.sqrt(1-2.0*(e0*e2-e1*e3)))
    psi = np.arctan2(2.0 * (e0 * e3 + e1 * e2), e0**2.0 + e1**2.0 - e2**2.0 - e3**2.0)
    return phi, theta, psi


##### Case 1 ######


mav = ac(p)

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
T_p =  -12.43072534597213
Q_p = -0.49879620097737787 


forcesAndMoments = mav.get_aero_forces(state, [0, 0, 0], [elevator, aileron, rudder])
forcesAndMoments += mav.get_gravity(state)
forcesAndMoments += mav.get_thrust(T_p, Q_p)

print("Forces and Moments : Case 1" , "\n")
print("fx: " , forcesAndMoments.item(0))
print("fy: " , forcesAndMoments.item(1))
print("fz: " , forcesAndMoments.item(2))
print("Mx: " , forcesAndMoments.item(3))
print("My: " , forcesAndMoments.item(4))
print("Mz: " , forcesAndMoments.item(5) , "\n\n")

x_dot = mav.x_dot(0, state, forcesAndMoments)
print("State Derivatives : Case 1", "\n")
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
print("    r_dot: ", x_dot.item(11) , "\n\n\n")

# Answers for Case 1:

# Propeller Forces and Torque 

# T_p:  -12.43072534597213
# Q_p:  -0.49879620097737787 


# Forces and Moments : Case 1 

# fx:  -12.109717001006562
# fy:  0.20707328125000002
# fz:  63.44373750624077
# Mx:  0.5063701133123779
# My:  8.75643373378125
# Mz:  -0.21774997963125006 


# State Derivatives : Case 1 

# north_dot:  25.0
# east_dot:  0.0
# down_dot:  0.0
#    u_dot:  -1.1008833637278692
#    v_dot:  0.01882484375
#    w_dot:  5.767612500567343
#   e0_dot:  -0.0
#   e1_dot:  0.0
#   e2_dot:  0.0
#   e3_dot:  0.0
#    p_dot:  0.6021690003674433
#    q_dot:  7.714919589234582
#     r_dt:  -0.08257466286924951 



##### Case 2 ######

mav = ac(p)

elevator = -0.15705144
aileron = 0.01788999
rudder = 0.01084654
throttle = 1.

# quat = [x, y, z, w]
quaternion = [2.47421558e-01, 6.56821468e-02, 2.30936730e-01, 9.38688796e-01]
rotation = R.from_quat(quaternion)
euler = rotation.as_euler('ZYX', degrees=False)

print(f"Euler Angles: {euler}")
"""
quaternion = np.array([[ 9.38688796e-01],
[ 2.47421558e-01],
[ 6.56821468e-02],
[ 2.30936730e-01]]).flatten()

euler = quaternion_to_euler(quaternion)

print(f"Euler Angles: {euler}")
"""
state = np.array([[ 6.19506532e+01],
 [ 2.22940203e+01],
 [-1.10837551e+02],
 [ 2.73465947e+01],
 [ 6.19628233e-01],
 [ 1.42257772e+00],
 [euler[2]],
 [euler[1]],
 [euler[0]],
#  [ 9.38688796e-01],
#  [ 2.47421558e-01],
#  [ 6.56821468e-02],
#  [ 2.30936730e-01],
 [ 4.98772167e-03],
 [ 1.68736005e-01],
 [ 1.71797313e-01]]).flatten()

T_p =  37.7794805541605
Q_p =  1.8098467397878482 

forcesAndMoments = mav.get_aero_forces(state, [0, 0, 0], [elevator, aileron, rudder])
forcesAndMoments += mav.get_gravity(state)
forcesAndMoments += mav.get_thrust(T_p, Q_p)

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
print("    r_dot: ", x_dot.item(11) , "\n\n\n")

# Answers for Case 2:
# Propeller Forces and Torque 

# T_p:  37.7794805541605
# Q_p:  1.8098467397878482 


# Forces and Moments : Case 2 

# fx:  36.99938467421735
# fy:  54.13991070468528
# fz:  46.97294443218653
# Mx:  1.6030203500067555
# My:  5.982053219886495
# Mz:  -1.1805441645292776 


# State Derivatives : Case 2 

# north_dot:  24.283238643486627
# east_dot:  12.605130052025968
# down_dot:  1.2957327060769266
#    u_dot:  3.2299908091423797
#    v_dot:  0.2308339966235602
#    w_dot:  8.881532282520418
#   e0_dot:  -0.025995661302161892
#   e1_dot:  -0.011500703223228347
#   e2_dot:  0.05851804333262313
#   e3_dot:  0.10134276693843723
#    p_dot:  1.8427420637214973
#    q_dot:  5.2743652738342774
#     r_dt:  -0.5471458931221012 