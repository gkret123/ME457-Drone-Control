import numpy as np
# import design_projects.chap05.model_coef as TF
import parameters.aerosonde_parameters as MAV
import models.model_coef as TF


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



gravity = MAV.gravity  # gravity constant
rho = MAV.rho  # density of air
sigma = 0.05  # low pass filter gain for derivative
Va0 = TF.Va_trim

# Ch. 6

#----------roll loop-------------
# get transfer function data for delta_a to phi
wn_roll = 10.0 #20 #7
zeta_roll = 0.707
roll_kp = wn_roll**2/TF.a_phi2
roll_kd = (2.0 * zeta_roll * wn_roll - TF.a_phi1) / TF.a_phi2

# print('roll_kp =', roll_kp)


#----------course loop-------------
wn_course = wn_roll / 20.0
zeta_course = 1.0
course_kp = 2.0 * zeta_course * wn_course * Va0 / gravity
course_ki = wn_course**2 * Va0 / gravity

#----------yaw damper-------------
yaw_damper_p_wo = 0.45  # (old) 1/0.5
yaw_damper_kr = 0.2  # (old) 0.5 

#----------pitch loop-------------
wn_pitch = 15.0 #24.0
zeta_pitch = 0.707
pitch_kp = (wn_pitch**2 - TF.a_theta2) / TF.a_theta3
pitch_kd = (2.0 * zeta_pitch * wn_pitch - TF.a_theta1) / TF.a_theta3
K_theta_DC = pitch_kp * TF.a_theta3 / (TF.a_theta2 + pitch_kp * TF.a_theta3)

#----------altitude loop-------------
wn_altitude = wn_pitch / 30.0
zeta_altitude = 1.0
altitude_kp = 2.0 * zeta_altitude * wn_altitude / K_theta_DC / Va0
altitude_ki = wn_altitude**2 / K_theta_DC / Va0
altitude_zone = 5.0  # moving saturation limit around current altitude

#---------airspeed hold using throttle---------------
wn_airspeed_throttle = 1.5 #3.0
zeta_airspeed_throttle = 2.0  # 0.707
airspeed_throttle_kp = (2.0 * zeta_airspeed_throttle * wn_airspeed_throttle - TF.a_V1) / TF.a_V2
airspeed_throttle_ki = wn_airspeed_throttle**2 / TF.a_V2

#----------climb rate hold using pitch-------------
climbrate_kp = 1.0
climbrate_ki = 0.0
climbrate_kd = 1.0
