# -*- coding: utf-8 -*-
"""
Created on Sat Jan 25 20:48:21 2025

@author: gabri
"""

import numpy as np


#Part A

P_COM = np.array([0,0,-10])
P_batt_body = np.array([0.2, 0, 0])

#euler angles

yaw = np.deg2rad(2) 
roll = np.deg2rad(20)
pitch = np.deg2rad(10)


R3 = np.array([[np.cos(yaw), -np.sin(yaw), 0],
              [np.sin(yaw), np.cos(yaw), 0],
              [0, 0, 1]])

R2 = np.array([[np.cos(pitch), 0, np.sin(pitch)],
              [0, 1, 0],
              [-np.sin(pitch), 0, np.cos(pitch)]])

R1 = np.array([[1, 0, 0],
              [0, np.cos(roll), -np.sin(roll)],
              [0, np.sin(roll), np.cos(roll)]])

R_eb = R3 @ R2 @ R1
print(f"Rotation matrix from earth to body: \n {R_eb} \n" )

P_batt_earth = list(P_COM + R_eb @ P_batt_body )


print(f"Coordinates of battery wrt earth: {P_batt_earth} m \n")

#Part B

V_g_b = np.array([15, 1, 0.5]) #m/s

V_g_e = R_eb @ V_g_b 

print(f"The velocity of the body in the ground frame is {V_g_e} m/s \n")

#Part C

flight_path_ang = np.arcsin(V_g_e[2]/(np.sqrt(V_g_e[0]**2 + V_g_e[1]**2 + V_g_e[2]**2)))

print(f"The flight-path angle is {np.rad2deg(flight_path_ang)} Degrees \n")

#Part D
AoA = np.arctan(V_g_b[2]/V_g_b[0])

print(f"The Angle of Attack is {np.rad2deg(AoA)} Degrees \n")

#Part E

course_angle = (np.arctan2(V_g_e[1], V_g_e[0]))

heading_angle = yaw

print(f"The Course Angle is {np.rad2deg(course_angle)} Degrees \n")
print(f"The Heading Angle is {np.rad2deg(heading_angle)} Degrees")
