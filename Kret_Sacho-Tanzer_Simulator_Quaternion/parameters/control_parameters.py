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
wn_roll = 0
zeta_roll = 0
roll_kp = 3  # K_u = 8, T_u = 0.2 s
roll_kd = .6

#----------course loop-------------
wn_course = 1
zeta_course = 0.7
course_kp = 4.5  # K_u = 10, T_u = 1 s
course_ki = 5.4

#----------yaw damper-------------
yaw_damper_p_wo = 1
yaw_damper_kr = 5

#----------pitch loop-------------
wn_pitch = 0
zeta_pitch = 0 
pitch_kp = 1
pitch_kd = 0.16
K_theta_DC = 0

#----------altitude loop-------------
wn_altitude = 0
zeta_altitude = 0
altitude_kp = 10
altitude_ki = 1
altitude_zone = 10

#---------airspeed hold using throttle---------------
wn_airspeed_throttle = 0
zeta_airspeed_throttle = 0
airspeed_throttle_kp = 0
airspeed_throttle_ki = 0
