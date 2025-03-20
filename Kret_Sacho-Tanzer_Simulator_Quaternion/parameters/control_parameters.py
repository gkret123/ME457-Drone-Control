import numpy as np
# import design_projects.chap05.model_coef as TF
import parameters.aerosonde_parameters as MAV


#### TODO #####
gravity = MAV.gravity  # gravity constant
Va0 = 25 # TF.Va_trim
rho = 1.293 # kg/m3 # density of air (From google)
sigma = 1  # low pass filter gain for derivative

#----------roll loop-------------
# get transfer function data for delta_a to phi
wn_roll = 0.7
zeta_roll = 0.7
roll_kp = 0.7
roll_kd = 0.7

#----------course loop-------------
wn_course = 0.7
zeta_course = 0.7
course_kp = 0.7
course_ki = 0.7

#----------yaw damper-------------
yaw_damper_p_wo = 0.7
yaw_damper_kr = 0.7

#----------pitch loop-------------
wn_pitch = 0.7
zeta_pitch = 0.7 
pitch_kp = 0.7
pitch_kd = 0.7
K_theta_DC = 0.7

#----------altitude loop-------------
wn_altitude = 0.7
zeta_altitude = 0.7
altitude_kp = 0.7
altitude_ki = 0.7
altitude_zone = 0.7

#---------airspeed hold using throttle---------------
wn_airspeed_throttle = 0.7
zeta_airspeed_throttle = 0.7
airspeed_throttle_kp = 0.7
airspeed_throttle_ki = 0.7
