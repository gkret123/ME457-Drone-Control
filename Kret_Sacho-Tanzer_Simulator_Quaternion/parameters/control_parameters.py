import numpy as np
# import design_projects.chap05.model_coef as TF
import parameters.aerosonde_parameters as MAV


#### TODO #####
gravity = MAV.gravity  # gravity constant
Va0 = 36 # TF.Va_trim
Va_trim = 36 # TF.Va_trim
alpha_trim = 0.0 # TF.alpha_trim
delta_trim = 1.4 # TF.delta_trim
theta_trim = 0.0 # TF.theta_trim
rho = 1.293 # kg/m3 # density of air (From google)
sigma = 1  # low pass filter gain for derivative

a_phi_1 = -1/2 * MAV.rho * (Va_trim ** 2) *  MAV.S_wing * MAV.b * MAV.C_p_p * MAV.b / (2 * Va_trim)
a_phi_2 = 1/2 * MAV.rho * (Va_trim ** 2) *  MAV.S_wing * MAV.b * MAV.C_p_delta_a
a_theta_1 = -((MAV.rho * Va_trim ** 2 * MAV.c * MAV.S_wing) / (2 * MAV.Jy)) * MAV.C_m_q
a_theta_2 = -((MAV.rho * Va_trim ** 2 * MAV.c * MAV.S_wing) / (2 * MAV.Jy)) * MAV.C_m_alpha
a_theta_3 = ((MAV.rho * Va_trim ** 2 * MAV.c * MAV.S_wing) / (2 * MAV.Jy)) * MAV.C_m_delta_e

a_v_1 = ((MAV.rho * Va_trim * MAV.S_wing) / MAV.mass) * \
           (MAV.C_D_0 + MAV.C_D_alpha * alpha_trim + MAV.C_D_delta_e * delta_trim) - \
            (MAV.rho * MAV.S_prop * MAV.C_prop * Va_trim / MAV.mass)
           #dT_dVa(delta_trim, Va_trim) / MAV.mass # AT : Not sure where I got this from (slides?) but the dt_dtva stuff is incorrect
    #a_V2 = dT_ddelta_t(mav, delta_trim, Va_trim) / MAV.mass
a_v_2 = (MAV.rho * MAV.S_prop * MAV.C_prop * MAV.k_motor ** 2 ) / MAV.mass
a_v_3 = MAV.gravity * np.cos(theta_trim - alpha_trim)


#----------roll loop-------------
# get transfer function data for delta_a to phi
wn_roll = 20
zeta_roll = 0.7
roll_kp = wn_roll**2/a_phi_2
roll_kd = (2*zeta_roll*wn_roll-a_phi_1)/a_phi_2

#----------course loop-------------
wn_course = 1
zeta_course = 0.7
course_kp = 2*zeta_course*wn_course*Va0/gravity  # maybe Va0 should be Vg
course_ki = wn_course**2*Va0/gravity

#----------yaw damper-------------
yaw_damper_p_wo = 1
yaw_damper_kr = 1

#----------pitch loop-------------
wn_pitch = 20
zeta_pitch = 0.7
pitch_kp = (wn_pitch**2-a_theta_2)/a_theta_3
pitch_kd = (2*zeta_pitch*wn_pitch-a_theta_1)/a_theta_3
K_theta_DC = pitch_kp*a_theta_3/(a_theta_2+pitch_kp*a_theta_3)

#----------altitude loop-------------
wn_altitude = 1
zeta_altitude = 0.7
altitude_kp = 2*zeta_altitude*wn_altitude/(K_theta_DC*Va0)  # maybe Va0 should be Vg
altitude_ki = wn_altitude**2/(K_theta_DC*Va0)
altitude_zone = 5

#---------airspeed hold using throttle---------------
wn_airspeed_throttle = 10
zeta_airspeed_throttle = 0.7
airspeed_throttle_kp = 2(*zeta_airspeed_throttle*wn_airspeed_throttle-a_v_1)/a_v_2
airspeed_throttle_ki = wn_airspeed_throttle**2/a_v_2