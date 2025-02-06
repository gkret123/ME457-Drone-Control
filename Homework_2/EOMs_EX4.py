# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 14:45:37 2025

@author: Adin Sacho-Tanzer & Gabriel Kret
"""

import numpy as np
import matplotlib.pyplot as plt
import integrators_HW2 as intg
 
#state: x = [p_n, p_e, p_d, u, v, w, phi, theta, psi, p, q, r]

#u,v,w is the velocity in the body frame
#p,q,r is the angular velocity in the body frame
#p_n, p_e, p_d is the position in the NED frame
#phi, theta, psi is the euler angles

# define matrices

A = np.array([[np.cos(theta)*np.cos(psi), np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi)],
             [np.cos(theta)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi), np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi)],
             [-np.sin(theta), np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(theta)]])

B = np.array([[r*v - q*w],
              [p*w - r*u],
              [q*u - p*v]])

C = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                [0, np.cos(phi), -np.sin(phi)],
                [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

D = np.array([[T1*p*q - T2*q*r],
              [T5*p*r - T6*(p**2 - r**2)],
              [T7*p*q - T1*q*r]])

E = np.array([[T3*l + T4*n],
              [1/J_y*m],
              [T4*l + T8*n]])

T1 = J_xz*(J_xx - J_yy + J_zz)/T
T2 = (J_zz*(J_zz - J_yy) + J_xz**2)/T
T3 = J_zz/T
T4 = (J_zz - J_xx)/T
T5 = (J_xx - J_yy)/T
T6 = J_xz/T
T7 = (J_zz - J_xx)/T
T8 = J_xx*(J_xx - J_yy + J_zz)/T
T = J_x*J_z - J_xz**2