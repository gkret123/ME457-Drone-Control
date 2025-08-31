import numpy as np
from tools.rotations import euler_to_quaternion, quaternion_to_euler

######################################################################################
                #   NOTE: These are parameters for Cessna 172. Source (unless otherwise noted): https://www.researchgate.net/figure/Stability-derivatives-for-Cessna-172_fig11_309468360
######################################################################################
print("Using Cessna 172 parameters")

######################################################################################
                #   Initial Conditions
######################################################################################
#   Initial conditions for MAV
north0 = 0.  # initial north position
east0 = 0.  # initial east position
down0 = -0.0  # initial down position
u0 = 60.  # initial velocity along body x-axis
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


######################################################################################
                #   Physical Parameters
######################################################################################
mass = 1043.3 #kg
Jx = 1285.3 #kg m^2
Jy = 1824.9 #kg m^2
Jz = 2666.9 #kg m^2
Jxz = 0.0 #kg m^2  # NOTE: this is 0 from the source, but feels sketchy
S_wing = 16.1651 # m^2
b = 10.9118 # m
c = 1.4935  # m

rho = 1.2682
AR = (b**2) / S_wing
gravity = 9.81

# S_prop = 0.2027
# e = 0.9


######################################################################################
                #   Longitudinal Coefficients
######################################################################################
C_L_0 = 0.31
C_D_0 = 0.031
C_m_0 = -0.015
C_L_alpha = 5.143
C_D_alpha = 0.13
C_m_alpha = -0.89
C_L_q = 3.9
C_D_q = 0.0
C_m_q = -12.4
C_L_delta_e = 0.43
C_D_delta_e = 0.06
C_m_delta_e = -1.28

# M = 50.0
# alpha0 = 0.47
# epsilon = 0.16
# C_D_p = 0.043


######################################################################################
                #   Lateral Coefficients
######################################################################################
C_Y_0 = 0.0  # NOTE: this wasn't in the source, but was 0 for aerosonde
C_ell_0 = 0.0
C_n_0 = 0.0
C_Y_beta = -0.31
C_ell_beta = -0.089
C_n_beta = 0.065
C_Y_p = -0.037
C_ell_p = -0.47
C_n_p = -0.03
C_Y_r = 0.21
C_ell_r = 0.096
C_n_r = -0.099
C_Y_delta_a = 0.0
C_ell_delta_a = -0.178
C_n_delta_a = -0.053
C_Y_delta_r = 0.187
C_ell_delta_r = 0.0147
C_n_delta_r = -0.0657

######################################################################################
                #   Propeller thrust / torque parameters (see addendum by McLain)  TODO: all of these are wrong for cessna 172
######################################################################################
max_power = 134000  # engine max power in watts (source: https://en.wikipedia.org/wiki/Cessna_172#:~:text=The%20Cessna%20172S%20was%20introduced,180%20horsepower%20(134%20kW).
eta = 0.8  # apparently typical according to google
A_p = 1.132 # source: https://archive.aoe.vt.edu/lutze/AOE3104/thrustmodels.pdf
B_p = 0.132



# Prop parameters
# D_prop = 1.93     # prop diameter in m  # SOURCE: https://www.researchgate.net/publication/353752543_Cessna_172_Flight_Simulation_Data#:~:text=Number%20of%20blades%202%20

# # Motor parameters
# KV_rpm_per_volt = 145.                            # Motor speed constant from datasheet in RPM/V
# KV = (1. / KV_rpm_per_volt) * 60. / (2. * np.pi)  # Back-emf constant, KV in V-s/rad
# KQ = KV                                           # Motor torque constant, KQ in N-m/A  NOTE: needed
# R_motor = 0.042              # ohms
# i0 = 1.5                     # no-load (zero-torque) current (A)


# # Inputs
# ncells = 12.
# V_max = 3.7 * ncells  # max voltage for specified number of battery cells

# # Coeffiecients from prop_data fit
# C_Q2 = 0.0109
# C_Q1 = -0.0088
# C_Q0 = 0.00635
# C_T2 = -0.0463
# C_T1 = -0.0195
# C_T0 = 0.045


######################################################################################
                #   Calculation Variables
######################################################################################
#   gamma parameters pulled from page 36 (dynamics)
gamma = Jx * Jz - (Jxz**2)
gamma1 = (Jxz * (Jx - Jy + Jz)) / gamma
gamma2 = (Jz * (Jz - Jy) + (Jxz**2)) / gamma
gamma3 = Jz / gamma
gamma4 = Jxz / gamma
gamma5 = (Jz - Jx) / Jy
gamma6 = Jxz / Jy
gamma7 = ((Jx - Jy) * Jx + (Jxz**2)) / gamma
gamma8 = Jx / gamma

#   C values defines on pag 62
C_p_0         = gamma3 * C_ell_0      + gamma4 * C_n_0
C_p_beta      = gamma3 * C_ell_beta   + gamma4 * C_n_beta
C_p_p         = gamma3 * C_ell_p      + gamma4 * C_n_p
C_p_r         = gamma3 * C_ell_r      + gamma4 * C_n_r
C_p_delta_a    = gamma3 * C_ell_delta_a + gamma4 * C_n_delta_a
C_p_delta_r    = gamma3 * C_ell_delta_r + gamma4 * C_n_delta_r
C_r_0         = gamma4 * C_ell_0      + gamma8 * C_n_0
C_r_beta      = gamma4 * C_ell_beta   + gamma8 * C_n_beta
C_r_p         = gamma4 * C_ell_p      + gamma8 * C_n_p
C_r_r         = gamma4 * C_ell_r      + gamma8 * C_n_r
C_r_delta_a    = gamma4 * C_ell_delta_a + gamma8 * C_n_delta_a
C_r_delta_r    = gamma4 * C_ell_delta_r + gamma8 * C_n_delta_r
