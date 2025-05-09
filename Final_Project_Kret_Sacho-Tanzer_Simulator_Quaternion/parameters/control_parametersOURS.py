import numpy as np
# import design_projects.chap05.model_coef as TF
import parameters.aerosonde_parameters as MAV


#### TODO #####
gravity = MAV.gravity  # gravity constant
Va0 = 25
Va_trim = 25
# from ariel:
alpha_trim = 0.0501
delta_trim = 0.67
theta_trim = 0.0501
# from luchtenberg:
# alpha_trim = 0.0478
# theta_trim = 0.222
# delta_trim = 0.8577

rho = MAV.rho # kg/m3 # density of air (From google)
sigma = 0.05  # low pass filter gain for derivative

a_phi_1 = -1/2 * MAV.rho * (Va_trim ** 2) *  MAV.S_wing * MAV.b * MAV.C_p_p * MAV.b / (2 * Va_trim)
a_phi_2 = 1/2 * MAV.rho * (Va_trim ** 2) *  MAV.S_wing * MAV.b * MAV.C_p_delta_a
a_theta_1 = -((MAV.rho * Va_trim ** 2 * MAV.c * MAV.S_wing) / (2 * MAV.Jy)) * MAV.C_m_q
a_theta_2 = -((MAV.rho * Va_trim ** 2 * MAV.c * MAV.S_wing) / (2 * MAV.Jy)) * MAV.C_m_alpha
a_theta_3 = ((MAV.rho * Va_trim ** 2 * MAV.c * MAV.S_wing) / (2 * MAV.Jy)) * MAV.C_m_delta_e



k_motor = 80  # C_prop, K_motor were originally from MAV, but aren't in our version. I just took these numbers from https://github.com/destrospooder/me457_dronecontrol/blob/main/parameters/aerosonde_parameters.py
C_prop = 1  # but they should possibly be calculated somehow

# a_theta_1, a_v_1, a_v_2

a_v_1 = ((MAV.rho * Va_trim * MAV.S_wing) / MAV.mass) * \
           (MAV.C_D_0 + MAV.C_D_alpha * alpha_trim + MAV.C_D_delta_e * delta_trim) - \
            (MAV.rho * MAV.S_prop * C_prop * Va_trim / MAV.mass)
           #dT_dVa(delta_trim, Va_trim) / MAV.mass # AT : Not sure where I got this from (slides?) but the dt_dtva stuff is incorrect
    #a_V2 = dT_ddelta_t(mav, delta_trim, Va_trim) / MAV.mass
a_v_2 = (MAV.rho * MAV.S_prop * C_prop * k_motor ** 2 ) / MAV.mass
a_v_3 = MAV.gravity * np.cos(theta_trim - alpha_trim)

# # from luchtenburg:
# a_phi_1 = 22.628851
# a_phi_2 = 130.883678
# a_theta_1 = 5.294738
# a_theta_2 = 99.947422
# a_theta_3 = -36.112390
# a_v_1 = 0.052559
# a_v_2 = 10.698657
# a_v_3 = 9.651117


#----------roll loop-------------
# get transfer function data for delta_a to phi
wn_roll = 10.1 # 20
zeta_roll = 3.5 # 0.707
roll_kp = wn_roll**2/a_phi_2
roll_kd = (2*zeta_roll*wn_roll-a_phi_1)/a_phi_2

#----------course loop-------------
wn_course = wn_roll/20
zeta_course = 1
course_kp = 2*zeta_course*wn_course*Va0/gravity  # maybe Va0 should be Vg
course_ki = wn_course**2*Va0/gravity

#----------yaw damper-------------
yaw_damper_p_wo = 10# 0.45
yaw_damper_kr = 0.2

#----------pitch loop-------------
wn_pitch = 10.25 # 24
zeta_pitch = 2.2 # 0.7
pitch_kp = (wn_pitch**2-a_theta_2)/a_theta_3
pitch_kd = (2*zeta_pitch*wn_pitch-a_theta_1)/a_theta_3
K_theta_DC = pitch_kp*a_theta_3/(a_theta_2+pitch_kp*a_theta_3)

#----------altitude loop-------------
wn_altitude = wn_pitch/30
zeta_altitude = 1
altitude_kp = 2*zeta_altitude*wn_altitude/(K_theta_DC*Va0)  # maybe Va0 should be Vg
altitude_ki = wn_altitude**2/(K_theta_DC*Va0)
altitude_zone = 5

#---------airspeed hold using throttle---------------
wn_airspeed_throttle = 3
zeta_airspeed_throttle = 2
airspeed_throttle_kp = (2*zeta_airspeed_throttle*wn_airspeed_throttle-a_v_1)/a_v_2
airspeed_throttle_ki = wn_airspeed_throttle**2/a_v_2