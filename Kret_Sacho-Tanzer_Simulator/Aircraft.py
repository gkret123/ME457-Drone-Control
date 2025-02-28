from EOMs_main import rigid_body
import numpy as np

class Aircraft(rigid_body):
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