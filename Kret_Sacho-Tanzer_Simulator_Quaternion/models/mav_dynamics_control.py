"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    - MavDynamics_control -> Aerodynamics
    
"""
import sys
import os
import matplotlib.pyplot as plt
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import numpy as np
#mav dynamics is the equivlent of rigid body
from models.mav_dynamics import MavDynamics as MavDynamicsForces
# load message types
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
import parameters.aerosonde_parameters as MAV
import plotter.plot_results as plot
from tools.rotations import quaternion_to_rotation, quaternion_to_euler

#
class MavDynamics(MavDynamicsForces):
    
    def __init__(self, Ts):
        super().__init__(Ts)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        #may need to update in the future
        self._Va = MAV.u0
        #AoA and sideslip
        self._alpha = 0
        self._beta = 0
        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())
        # update the message class for the true state
        self._update_true_state()


    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)
        super()._rk4_step(forces_moments)
        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _update_velocity_data(self, wind=np.zeros((6,1))):
    
        steady_state = wind[0:3]
        gust = wind[3:6]

        ##### TODO #####
        # convert steady-state wind vector from world to body frame
        # wind_body = 
        # add the gust 
        # wind_body += 
        # convert total wind to world frame
        # self._wind = 

        # velocity vector relative to the airmass ([ur , vr, wr]= ?)
        u_w, v_w, w_w = quaternion_to_rotation(self._state[6:10]).T @ self._wind  # because self._wind is in NED frame, need to convert to body frame before getting relative velocity
        V_ba = [self._state[3] - u_w, self._state[4] - v_w, self._state[5] - w_w]
        # compute airspeed (self._Va = ?)
        self._Va = np.linalg.norm(V_ba)
        # compute angle of attack (self._alpha = ?)
        self._alpha = np.arctan2(V_ba[2], V_ba[0])
        # compute sideslip angle (self._beta = ?)
        self._beta = np.arctan2(V_ba[1], np.sqrt(V_ba[0]**2 + V_ba[2]**2))
    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        ##### TODO ######
        # extract states (phi, theta, psi, p, q, r)
        u = self._state[3]
        v = self._state[4]
        w = self._state[5]
        
        p = self._state[10]
        q = self._state[11]
        r = self._state[12]
        # compute gravitational forces ([fg_x, fg_y, fg_z])
        R_0b = quaternion_to_rotation(self._state[6:10])
        fg = R_0b.T @ np.array([[0], [0], [MAV.mass * MAV.gravity]])
        # compute Lift and Drag coefficients (CL, CD)
        
        F_lift = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * (MAV.C_L_0 + MAV.C_L_alpha * self._alpha + MAV.C_L_q * MAV.c * q / (2 * self._Va) + MAV.C_L_delta_e * delta.elevator) 
        F_drag = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * (MAV.C_D_0 + MAV.C_D_alpha * self._alpha + MAV.C_D_q * MAV.c * q / (2 * self._Va) + MAV.C_D_delta_e * delta.elevator)

        # propeller thrust and torque
        thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta.throttle)

        # compute longitudinal forces in body frame (fx, fz)
        f_x, f_z = np.array([[np.cos(self._alpha), -np.sin(self._alpha)], [np.sin(self._alpha),  np.cos(self._alpha)]]).squeeze() @ np.array([-F_drag, -F_lift]).squeeze()
        # compute lateral forces in body frame (fy)
        f_y = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * (MAV.C_Y_0 + MAV.C_Y_beta * self._beta + MAV.C_Y_p * MAV.b * p / (2 * self._Va) + MAV.C_Y_r * MAV.b * r / (2 * self._Va) + MAV.C_Y_delta_a * delta.aileron + MAV.C_Y_delta_r * delta.rudder)
        # compute longitudinal torque in body frame (My)
        m = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * MAV.c * (MAV.C_m_0 + MAV.C_m_alpha * self._alpha + MAV.C_m_q * MAV.c * q / (2 * self._Va) + MAV.C_m_delta_e * delta.elevator)
        # compute lateral torques in body frame (Mx, Mz)
        l = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * MAV.b * (MAV.C_ell_0 + MAV.C_ell_beta * self._beta + MAV.C_ell_p * MAV.b * p / (2 * self._Va) + MAV.C_ell_r * MAV.b * r / (2 * self._Va) + MAV.C_ell_delta_a * delta.aileron + MAV.C_ell_delta_r * delta.rudder)
        n = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing * MAV.b * (MAV.C_n_0 + MAV.C_n_beta * self._beta + MAV.C_n_p * MAV.b * p / (2 * self._Va) + MAV.C_n_r * MAV.b * r / (2 * self._Va) + MAV.C_n_delta_a * delta.aileron + MAV.C_n_delta_r * delta.rudder)
        
        f_x += thrust_prop + fg[0]
        f_y += fg[1]
        f_z += fg[2]
        l -= torque_prop

        forces_moments = np.array([f_x, f_y, f_z, l, m, n]).T
        return forces_moments

    def _motor_thrust_torque(self, Va, delta_t):
        # compute thrust and torque due to propeller from slides ch 4
        #map 0-1 throttle to voltage
        V_in = MAV.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = MAV.C_Q0 * MAV.rho * np.power(MAV.D_prop , 5)/((2.* np.pi)**2)
        b = (MAV.C_Q1 * MAV.rho * np.power(MAV.D_prop , 4) / (2. * np.pi )) * self._Va + MAV.KQ**2/MAV.R_motor
        c = MAV.C_Q2 * MAV.rho * np.power (MAV.D_prop , 3) * self._Va**2 - (MAV.KQ / MAV.R_motor) * V_in + MAV.KQ * MAV.i0

        # Consider only positive root for Omega
        Omega_op = (-b + np.sqrt(b**2-4*a*c)) / (2.* a )

        # compute advance ratio,
        J_op = 2 * np.pi * self._Va/(Omega_op * MAV.D_prop)
        # compute dimentionless coefficients of thrust and torque
        C_T = MAV.C_T2 * J_op ** 2 + MAV.C_T1 * J_op + MAV.C_T0
        C_Q = MAV.C_Q2 * J_op **2 + MAV.C_Q1 * J_op + MAV.C_Q0
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        thrust_prop = MAV.rho * n**2 * np.power(MAV.D_prop , 4) * C_T
        torque_prop = MAV.rho * n**2 * np.power(MAV.D_prop , 5) * C_Q

        return thrust_prop, torque_prop

    def _update_true_state(self):
        # rewrite this function because we now have more information
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = 0
        self.true_state.by = 0
        self.true_state.bz = 0
        self.true_state.camera_az = 0
        self.true_state.camera_el = 0

#make a plot of the results by repeatedly calling the update function
def main():
    # Initialize the model, wind, and control inputs.
        # Simulation settings.
    sim_time = 0.0
    Ts = 0.01
    sim_end_time = 10.0
    mav = MavDynamics(Ts)
    wind = np.array([[0.], [0.], [0.]])
    delta = MsgDelta()
    
    # Initialize history lists for time and state.
    time_array = []
    state_array = []
    
    # Simulation loop.
    while sim_time < sim_end_time:
        # Update the model.
        mav.update(delta, wind)
        
        # Record current time and state.
        time_array.append(sim_time)
        # Convert mav.true_state to a numpy array in the expected order:
        # [north, east, -altitude, Va, v, w, phi, theta, psi, p, q, r]
        state_array.append(np.array([
            mav.true_state.north,
            mav.true_state.east,
            -mav.true_state.altitude,  # convert altitude to down coordinate
            mav.true_state.Va,         # assuming Va corresponds to u
            0,  # Replace with actual v when it becomes available later in the course
            0,  # Replace with actual w when it becomes available later in the course
            mav.true_state.phi,
            mav.true_state.theta,
            mav.true_state.psi,
            mav.true_state.p,
            mav.true_state.q,
            mav.true_state.r
        ]))
        
        #advance timestep
        sim_time += Ts
    # Once simulation ends, create the plots using the collected data.
    plot.plot_results(time_array, state_array, title="Final Flight Dynamics")
    return time_array, state_array

if __name__ == "__main__":
    main()