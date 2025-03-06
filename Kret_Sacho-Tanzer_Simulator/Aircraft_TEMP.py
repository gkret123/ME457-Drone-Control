from EOMs_main import rigid_body
import numpy as np
from scipy.spatial.transform import Rotation as R
#from scipy.spatial.transform.rotation import Quaternion2Rotation, Quaternion2Euler
"""
class Aircraft(rigid_body):
REMOVED, REPLACEMENT FUNCTION BELOW
    def __init__(self, parameters, gravity=True):  # in future: wind, controller are parameters
        super().__init__(parameters, gravity)

    def get_aero_forces(self, state, wind, deflections):
        # Whatever wind vector we receive should be expressed in the body frame
        # Deflections = delta_e, delta_a, delta_r
        delta_e, delta_a, delta_r = deflections

        
        u = state[3]
        v = state[4]
        w = state[5]
        
        p = state[9]
        q = state[10]
        r = state[11]

        u_wind = wind[0]
        v_wind = wind[1]
        w_wind = wind[2]
        
        V_a_body = np.array([u - u_wind, v - v_wind, w - w_wind])
        V_a_mag = np.linalg.norm(V_a_body)

        u_r = V_a_body[0]
        v_r = V_a_body[1]
        w_r = V_a_body[2]

        alpha = np.arctan2(w_r, u_r)
        beta = np.arctan2(v_r, np.sqrt(u_r**2 + w_r**2))

        print(f"AOA: {alpha}, Sideslip: {beta}")

        rho = self.rho
        S = self.S
        c = self.c
        b = self.b
        
        C_L_0 = self.C_L_0
        C_L_alpha = self.C_L_alpha
        C_L_q = self.C_L_q
        C_L_delta_e = self.C_L_delta_e
        C_D_0 = self.C_D_0
        C_D_alpha = self.C_D_alpha
        C_D_q = self.C_D_q
        C_D_delta_e = self.C_D_delta_e
        C_m_0 = self.C_m_0
        C_m_alpha = self.C_m_alpha
        C_m_q = self.C_m_q
        C_m_delta_e = self.C_m_delta_e

        # Longitudinal forces:
        
        F_lift = 0.5*rho*V_a_mag**2*S*(C_L_0 + C_L_alpha * alpha + C_L_q*c*q/(2*V_a_mag) + C_L_delta_e*delta_e)
        F_drag = 0.5*rho*V_a_mag**2*S*(C_D_0 + C_D_alpha * alpha + C_D_q*c*q/(2*V_a_mag) + C_D_delta_e*delta_e)
        m = 0.5*rho*V_a_mag**2*S*c*(C_m_0 + C_m_alpha * alpha + C_m_q*c*q/(2*V_a_mag) + C_m_delta_e*delta_e)

        f_x, f_z = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]]) @ np.array([-F_drag, -F_lift])

        # For symmetric aircraft: C_Y_0 = C_ell_0 = C_n_0 = 0
        
        # Lateral forces:
        f_y = 0.5*rho*V_a_mag**2*S*(self.C_Y_0 + self.C_Y_beta*beta + self.C_Y_p*b*p/(2*V_a_mag) + self.C_Y_r*b*r/(2*V_a_mag) + self.C_Y_delta_a*delta_a + self.C_Y_delta_r*delta_r)
        l = 0.5*rho*V_a_mag**2*S*b*(self.C_ell_0 + self.C_ell_beta*beta + self.C_ell_p*b*p/(2*V_a_mag) + self.C_ell_r*b*r/(2*V_a_mag) + self.C_ell_delta_a*delta_a + self.C_ell_delta_r*delta_r)
        n = 0.5*rho*V_a_mag**2*S*b*(self.C_n_0 + self.C_n_beta*beta + self.C_n_p*b*p/(2*V_a_mag) + self.C_n_r*b*r/(2*V_a_mag) + self.C_n_delta_a*delta_a + self.C_n_delta_r*delta_r)
        
        return np.array([f_x, f_y, f_z, l, m, n])


    def get_forces(self, t, x_rk4_history):
        # wind = wind(t)
        wind = np.array([0, 0, 0])  # In body frame
        # deflections = controller(t, x_rk4_history)
        deflections = np.array([0, 0, 0])
        
        U = self.get_aero_forces(x_rk4_history[-1], wind, deflections)  # +thrust
        return U

    def simulate(self, x0, t_start, t_stop, dt=0.1):
        return super().simulate(x0, self.get_forces, t_start, t_stop, dt)
    """

class Aircraft(rigid_body):
    def __init__(self, parameters):  # in future: wind, controller are parameters
        super().__init__(parameters)

    def get_aero_forces(self, state, wind, deflections):
        # Whatever wind vector we receive should be expressed in the body frame
        # Deflections = delta_e, delta_a, delta_r
        delta_e, delta_a, delta_r = deflections

        u = state[3]
        v = state[4]
        w = state[5]
        
        p = state[9]
        q = state[10]
        r = state[11]

        u_wind = wind[0]
        v_wind = wind[1]
        w_wind = wind[2]
        
        V_a_body = np.array([u - u_wind, v - v_wind, w - w_wind])
        V_a_mag = np.linalg.norm(V_a_body)
        
        # Prevent division by zero:
        if V_a_mag < 1e-6:
            V_a_mag = 1e-6

        u_r = V_a_body[0]
        v_r = V_a_body[1]
        w_r = V_a_body[2]

        alpha = np.arctan2(w_r, u_r)
        beta = np.arctan2(v_r, np.sqrt(u_r**2 + w_r**2))

        print(f"AOA: {alpha}, Sideslip: {beta}")

        rho = self.rho
        S = self.S
        c = self.c
        b = self.b
        
        C_L_0 = self.C_L_0
        C_L_alpha = self.C_L_alpha
        C_L_q = self.C_L_q
        C_L_delta_e = self.C_L_delta_e
        C_D_0 = self.C_D_0
        C_D_alpha = self.C_D_alpha
        C_D_q = self.C_D_q
        C_D_delta_e = self.C_D_delta_e
        C_m_0 = self.C_m_0
        C_m_alpha = self.C_m_alpha
        C_m_q = self.C_m_q
        C_m_delta_e = self.C_m_delta_e

        quaternion = [2.47421558e-01, 6.56821468e-02, 2.30936730e-01, 9.38688796e-01]
        rotation = R.from_quat(quaternion)
        phi, theta, psi = rotation.as_euler('ZYX', degrees=False)
        

        # compute gravitaional forces
        f_g = self.mass * self.g

        # compute Lift and Drag coefficients
        CL = C_L_0 + C_L_alpha * alpha
        CD = C_D_0 + C_D_alpha * alpha
        # compute Lift and Drag Forces
        F_lift = 0.5 * rho * V_a_mag**2 * S * (CL + 
            C_L_q * c / (2*V_a_mag) * q + 
            C_L_delta_e * delta_e)
        F_drag = 0.5 * rho * V_a_mag**2 * S * (CD + 
            C_D_q * c / (2*V_a_mag) * q + 
            C_D_delta_e * delta_e)

        #compute propeller thrust and torque
        thrust_prop, torque_prop = 37.779480554160, 1.8098467397878482 


        # compute longitudinal forces in body frame
        fx = -f_g * np.sin(theta) + np.cos(alpha)*(-F_drag) - np.sin(alpha)*(-F_lift) + thrust_prop
        fz = f_g * np.cos(theta) * np.cos(phi) + np.sin(alpha)*(-F_drag) + np.cos(alpha)*(-F_lift)

        # compute lateral forces in body frame
        fy = f_g * np.cos(theta) * np.sin(phi) + 0.5 * rho * V_a_mag**2 * S * (self.C_Y_0 + 
            self.C_Y_beta * beta + 
            self.C_Y_p * b / (2*V_a_mag) * p +
            self.C_Y_r * b / (2*V_a_mag) * r + 
            self.C_Y_delta_a * delta_a + 
            self.C_Y_delta_r * delta_r)

        # compute logitudinal torque in body frame
        My = 0.5 * rho * V_a_mag**2 * S * c * (C_m_0 + 
            C_m_alpha * alpha + 
            C_m_q * c / (2*V_a_mag) * q + 
            C_m_delta_e * delta_e)
        # compute lateral torques in body frame
        Mx = torque_prop + 0.5 * rho * V_a_mag**2 * S * b * (self.C_ell_0 + 
            self.C_ell_beta * beta + 
            self.C_ell_p * b / (2*V_a_mag) * p + 
            self.C_ell_r * b / (2*V_a_mag) * r + 
            self.C_ell_delta_a * delta_a + 
            self.C_ell_delta_r * delta_r)
        Mz = 0.5 * rho * V_a_mag**2 * S * b * (self.C_n_0 + self.C_n_beta * beta + self.C_n_p * b / (2*V_a_mag) * p + self.C_n_r * b / (2*V_a_mag) * r + self.C_n_delta_a * delta_a + self.C_n_delta_r * delta_r)

        return np.array([fx, fy, fz, Mx, My, Mz])
    
    def get_gravity(self, state):
        # --- NEW: Include gravity by transforming the inertial gravity vector to the body frame ---
        # Get Euler angles from state (assumed to be [phi, theta, psi] at indices 6, 7, 8)
        phi = state[6]
        theta = state[7]
        psi = state[8]

        R_0b = self.euler2rot(phi, theta, psi)
        # Define gravity in the inertial frame (NED: positive z is down)
        g_inertial = np.array([0, 0, self.g])
        # Transform gravity into the body frame:
        g_body = R_0b.T @ g_inertial
        # thrust_body =R_0b.T @ np.array([-12.43/2 , 0, 0])
        
        # Add the gravitational force (F = m * g) to the aerodynamic force vector:
        f_x = self.mass * g_body[0]
        f_y = self.mass * g_body[1]
        f_z = self.mass * g_body[2]
        return np.array([f_x, f_y, f_z, 0, 0, 0])  # gravity can never apply moments

    def get_thrust(self, T_p, Q_p):
        # T_p = -12.43072534597213
        # Q_p = -0.49879620097737787
        f_x = T_p
        # f_x = -3.45623356351  # this value of T_p gives the correct results for case 1
        f_y = 0
        f_z = 0
        M_x = -Q_p
        M_y = 0
        M_z = 0
        return np.array([f_x, f_y, f_z, M_x, M_y, M_z])
    
    def get_forces(self, t, x_rk4_history):
        # For simulation, you might eventually add wind and controller effects.
        # For now, we simply call get_aero_forces with zeros.
        wind = np.array([0, 0, 0])  # In body frame
        deflections = np.array([0, 0, 0])
        

        U = self.get_aero_forces(x_rk4_history[-1], wind, deflections) #+thrust
        return U

    def simulate(self, x0, t_start, t_stop, dt=0.1):
        # Override the rigid_body.simulate method to avoid double-adding gravity.
        # Since get_aero_forces already includes gravity when self.gravity is True,
        # we skip the extra gravity addition.
        import integrators as intg  # local import (or ensure it's imported at the top)
        rk4_integrator = intg.RK4(dt, self.x_dot)

        t_history = [t_start]
        x_rk4_history = [x0]

        x_rk4 = x0
        t = t_start

        while t < t_stop:
            U_temp = self.get_forces(t, x_rk4_history)
            x_rk4 = rk4_integrator.step(t, x_rk4, U_temp)
            t += dt
            t_history.append(t)
            x_rk4_history.append(x_rk4)
        
        return t_history, x_rk4_history
