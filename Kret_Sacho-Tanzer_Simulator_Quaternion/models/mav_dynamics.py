"""
ME-457 Drone COntrol
Spring 2025

@author: Adin Sacho-Tanzer & Gabriel Kret
The Propeller Heads


mavDynamics 
    - this file implements the EOMs (Equivlent to EOMs_main in Euler Implementation)
    - use unit quaternion for the attitude state
    - the conversion from Euler angles to quaternions can be handled at any time in the event we need to start working in euler angles
    - the state of the MAV is defined as:
        pn - North position in meters
        pe - East position in meters
        pd - Down position in meters
        u - velocity along body x-axis in meters/sec
        v - velocity along body y-axis in meters/sec
        w - velocity along body z-axis in meters/sec
        e0 - quaternion e0
        e1 - quaternion e1
        e2 - quaternion e2
        e3 - quaternion e3
        p - body frame roll rate in radians/sec
        q - body frame pitch rate in radians/sec
        r - body frame yaw rate in radians/sec
"""

import numpy as np
#needed to add this the manage the path and import other files properly
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# load message types
from message_types.msg_state import MsgState
import parameters.aerosonde_parameters as MAV
from tools.rotations import quaternion_to_rotation, quaternion_to_euler

class MavDynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.north0],  # (0)
                               [MAV.east0],   # (1)
                               [MAV.down0],   # (2)
                               [MAV.u0],    # (3)
                               [MAV.v0],    # (4)
                               [MAV.w0],    # (5)
                               [MAV.e0],    # (6)
                               [MAV.e1],    # (7)
                               [MAV.e2],    # (8)
                               [MAV.e3],    # (9)
                               [MAV.p0],    # (10)
                               [MAV.q0],    # (11)
                               [MAV.r0],    # (12)
                               [0],   # (13)
                               [0],   # (14)
                               ])
        # initialize true_state message
        self.true_state = MsgState()

    ###################################
    # public functions
    def update(self, forces_moments):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        self._rk4_step(forces_moments)
        # update the message class for the true state
        self._update_true_state()

    def external_set_state(self, new_state):
        self._state = new_state

    ###################################
    # private functions
    def _rk4_step(self, forces_moments):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._f(self._state[0:13], forces_moments)
        k2 = self._f(self._state[0:13] + time_step/2.*k1, forces_moments)
        k3 = self._f(self._state[0:13] + time_step/2.*k2, forces_moments)
        k4 = self._f(self._state[0:13] + time_step*k3, forces_moments)
        self._state[0:13] += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

    def _f(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """        
        # Extract the States
        north = state.item(0)
        east = state.item(1)
        down = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)

        # Extract Forces/Moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # Position Kinematics
        north_dot, east_dot, down_dot = np.array([[(e1**2 + e0**2 - e2**2 - e3**2), 2*(e1*e2 - e3*e0), 2*(e1*e3 + e2*e0)],
                           [2*(e1*e2 + e3*e0), (e2**2 + e0**2 - e1**2 - e3**2), 2*(e2*e3 - e1*e0)],
                            [2*(e1*e3 - e2*e0), 2*(e2*e3 +e1*e0), (e3**2 + e0**2 - e1**2 - e2**2)]]) @ np.array([[u, v, w]]).T

        # Position Dynamics
        u_dot = np.array([[r*v - q*w + fx/MAV.mass]]).T
        v_dot = np.array([[p*w - r*u + fy/MAV.mass]]).T
        w_dot = np.array([[q*u - p*v + fz/MAV.mass]]).T
        
        # rotational kinematics
        e0_dot, e1_dot, e2_dot, e3_dot = 0.5 * np.array([[0, -p, -q, -r],
                                                        [p, 0, r, -q],
                                                        [q, -r, 0, p],
                                                        [r, q, -p, 0]]) @ np.array([[e0, e1, e2, e3]]).T


        # rotatonal dynamics
        T = MAV.Jx * MAV.Jz - MAV.Jxz**2
        T1 = MAV.Jxz*(MAV.Jx - MAV.Jy + MAV.Jz)/T
        T2 = (MAV.Jz*(MAV.Jz - MAV.Jy) + MAV.Jxz**2)/T
        T3 = MAV.Jz/T
        T4 = MAV.Jxz/T
        T5 = (MAV.Jz-MAV.Jx)/MAV.Jy
        T6 = MAV.Jxz/MAV.Jy
        T7 = ((MAV.Jx - MAV.Jy)*MAV.Jx+MAV.Jxz**2)/T
        T8 = MAV.Jx/T
        
        p_dot, q_dot, r_dot =  np.array([[T1*p*q - T2*q*r],
                    [T5*p*r - T6*(p**2 - r**2)],
                    [T7*p*q - T1*q*r]]) +      np.array([[T3*l + T4*n],
                                                [1/MAV.Jy*m],
                                                [T4*l + T8*n]])
        

        # collect the derivative of the states and convert to scalars
        x_dot = np.array([[north_dot.item()],
                           [east_dot.item()],
                           [down_dot.item()],
                           [u_dot.item()],
                           [v_dot.item()],
                           [w_dot.item()],
                           [e0_dot.item()],
                           [e1_dot.item()],
                           [e2_dot.item()],
                           [e3_dot.item()],
                           [p_dot.item()],
                           [q_dot.item()],
                           [r_dot.item()]])
        #x_dot = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0]]).T
        return x_dot

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = 0
        self.true_state.alpha = 0
        self.true_state.beta = 0
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = 0
        self.true_state.gamma = 0
        self.true_state.chi = 0
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = 0
        self.true_state.we = 0
        self.true_state.bx = 0
        self.true_state.by = 0
        self.true_state.bz = 0
        self.true_state.camera_az = 0
        self.true_state.camera_el = 0