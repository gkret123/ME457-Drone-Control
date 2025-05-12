import numpy as np
from tools.rotations import quaternion_to_euler
import parameters.aerosonde_parameters as MAV


x_trim = np.array([2.21483670e-16, 1.38603097e-15, -4.55482139e-15, 6.27964546e+01, 
                   0.00000000e+00, -6.67304267e-01, 9.99985886e-01, 0.00000000e+00, 
                   -5.31301475e-03, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 
                   0.00000000e+00])
# Delta_trim: elevator= -0.0015578767550296886 aileron= -8.36815132248315e-09 rudder= -1.654545932142433e-08 throttle= 1.5114249917811482 azimuth_cmd= 0.0 elevation_cmd= 0.0 
Va_trim = 62.8
alpha_trim = np.arctan2(x_trim[5], x_trim[3])  
phi_trim, theta_trim, psi_trim = quaternion_to_euler(np.array([x_trim[6], x_trim[7], x_trim[8], x_trim[9]]))
delta_e_trim = -0.004330320590381889
delta_t_trim = 0.6953171466096343

a_phi1 = -1/2*MAV.rho*Va_trim**2*MAV.S_wing*MAV.b*MAV.C_p_p*MAV.b/(2*Va_trim)  # page 71 of pdf
a_phi2 = 1/2*MAV.rho*Va_trim**2*MAV.S_wing*MAV.b*MAV.C_p_delta_a
a_theta1 = -MAV.rho*Va_trim**2*MAV.c*MAV.S_wing*MAV.C_m_q*MAV.c/(2*MAV.Jy*2*Va_trim)  # page 75 of pdf
a_theta2 = -MAV.rho*Va_trim**2*MAV.S_wing*MAV.c*MAV.C_m_alpha/(2*MAV.Jy)
a_theta3 = MAV.rho*Va_trim**2*MAV.S_wing*MAV.c*MAV.C_m_delta_e/(2*MAV.Jy)
a_V1 = MAV.rho*Va_trim*MAV.S_wing*(MAV.C_D_0+MAV.C_D_alpha*alpha_trim+MAV.C_D_delta_e*delta_e_trim)/MAV.mass-1/MAV.mass*(-MAV.max_power*delta_t_trim*(MAV.A_p-MAV.B_p)*MAV.eta/Va_trim**2)  # page 79 of pdf
a_V2 = 1/MAV.mass*MAV.max_power*(MAV.A_p-MAV.B_p)*MAV.eta/Va_trim
a_V3 = MAV.gravity*np.cos(theta_trim-alpha_trim)

A_lon = np.array([
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000]])
B_lon = np.array([
    [0.000000, 0.000000],
    [0.000000, 0.000000],
    [0.000000, 0.000000],
    [0.000000, 0.000000],
    [0.000000, 0.000000]])
A_lat = np.array([
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000]])
B_lat = np.array([
    [0.000000, 0.000000],
    [0.000000, 0.000000],
    [0.000000, 0.000000],
    [0.000000, 0.000000],
    [0.000000, 0.000000]])
Ts = 0.010000
