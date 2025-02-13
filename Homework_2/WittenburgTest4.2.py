# -*- coding: utf-8 -*-
"""
Created on Thu Feb 13 14:44:32 2025

@author: Adin Sacho-Tanzer
"""

from EOMs_EX4 import rigid_body
import Parameters_Test as p
import numpy as np
from matplotlib import pyplot as plt

w_10 = 1
w_20 = 0
w_30 = 2

t_0 = 0

myPlane = rigid_body(p.mass, p.J_xx, p.J_yy, p.J_zz, p.J_xz, p.S, p.b, p.c, p.S_prop, p.rho, p.k_motor, p.k_T_p, p.k_Omega, p.e)
t, x = myPlane.simulate(np.array([0,0,0,0,0,0,0,0,0,w_10, w_20, w_30]), np.array([0,0,0,0,0,0]), t_0, 10, dt=0.1)
        #state: x = [p_n, p_e, p_d, u, v, w, phi, theta, psi, p, q, r]
        #inputs: U = [f_x, f_y, f_z, l, m, n]

t = np.array(t)
x = np.array(x)
fig, axs = plt.subplots(2, 2, figsize=(20, 15))

# Position in NED frame
axs[0,0].plot(t, x[:, 0:3])
axs[0,0].legend(["p_n (m)", "p_e (m)", "p_d (m)"])
axs[0,0].set_title("Position in NED Frame")
axs[0,0].set_xlabel("Time (s)")
axs[0,0].set_ylabel("Position (m)")

# Velocity in body frame
axs[0, 1].plot(t, x[:, 3:6])
axs[0,1].legend(["u (m/s)", "v (m/s)", "w (m/s)"])
axs[0,1].set_title("Linear Velocity in Body Frame")
axs[0,1].set_xlabel("Time (s)")
axs[0,1].set_ylabel("Velocity (m/s)")

# Euler angles
axs[1,0].plot(t, x[:, 6:9])
axs[1,0].legend(["phi (rad)", "theta (rad)", "psi (rad)"])
axs[1,0].set_title("Euler Angles")
axs[1,0].set_xlabel("Time (s)")
axs[1,0].set_ylabel("Angle (rad)")

# Angular velocities
axs[1,1].plot(t, x[:, 9:12])
axs[1,1].legend(["p (rad/s)", "q (rad/s)", "r (rad/s)"])
axs[1,1].set_title("Angular Velocities")
axs[1,1].set_xlabel("Time (s)")
axs[1,1].set_ylabel("Angular Velocity (rad/s)")

plt.tight_layout()
plt.show()

nu = w_30*(p.J_xx-p.J_zz)/p.J_xx
w1 = w_10*np.cos(nu*(t-t_0))+w_20*np.sin(nu*(t-t_0))
w2 = w_20*np.cos(nu*(t-t_0))-w_10*np.sin(nu*(t-t_0))

plt.figure(figsize=[10,6])
plt.title("Analytical and Numerical solutions to Wittenburg 4.2")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.plot(t,x[:, 9:12], label=["p_sim (rad/s)", "q_sim (rad/s)", "r_sim (rad/s)"])
plt.plot(t, np.array([w1, w2, [w_30 for _ in t]]).T, label=["p_analytical (rad/s)", "q_analytical (rad/s)", "r_analytical (rad/s)"], linestyle="dotted")
plt.legend(loc="best")
plt.show()
