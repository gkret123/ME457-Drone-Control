"""
observer2_test.py
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
"""
import sys
import numpy as np
from scipy import stats
sys.path.append('..')
import parameters.control_parameters as CTRL
import parameters.aerosonde_parameters as MAV
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors

class Observer:
    def __init__(self, ts_control, initial_state = MsgState(), initial_measurements = MsgSensors()):
        # initialized estimated state message
        self.estimated_state = initial_state
        # use alpha filters to low pass filter gyros and accels
        # alpha = Ts/(Ts + tau) where tau is the LPF time constant
        self.lpf_gyro_x = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)
        self.lpf_accel_x = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_x)
        self.lpf_accel_y = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_y)
        self.lpf_accel_z = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_z)
        # use alpha filters to low pass filter absolute and differential pressure
        self.lpf_abs = AlphaFilter(alpha=0.9, y0=initial_measurements.abs_pressure)
        self.lpf_diff = AlphaFilter(alpha=0.7, y0=initial_measurements.diff_pressure)
        # ekf for phi and theta
        self.attitude_ekf = EkfAttitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = EkfPosition()


    def update(self, measurement):

        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x - SENSOR.gyro_x_bias)
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y - SENSOR.gyro_y_bias)
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z - SENSOR.gyro_z_bias)

        # invert sensor model to get altitude and airspeed
        self.estimated_state.altitude = self.lpf_abs.update(measurement.abs_pressure) / (MAV.rho * MAV.gravity)
        self.estimated_state.Va = np.sqrt(2 / MAV.rho * self.lpf_diff.update(measurement.diff_pressure))

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(measurement, self.estimated_state)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state


class AlphaFilter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.alpha * self.y + (1 - self.alpha) * u
        return self.y


class EkfAttitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):
        self.Q = np.array([[0.05, 0], [0, 0.005]])
        self.Q_gyro = np.zeros((3, 3))
        self.R_accel = MAV.gravity * np.array([[SENSOR.accel_sigma**2, 0, 0],
                                               [0, SENSOR.accel_sigma**2, 0],
                                               [0, 0, SENSOR.accel_sigma**2]])
        self.N = 15  # number of prediction step per sample
        self.xhat = np.array([0, 0.05]) # initial state: phi, theta
        self.P = np.eye(2) * 0.2
        self.Ts = SIM.ts_simulation / self.N
        self.gate_threshold = stats.chi2.isf(q = 0.05, df = 3)

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        p = state.p
        q = state.q
        r = state.r

        phi = x[0]
        theta = x[1]

        f_ = np.array([p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta),
                       q * np.cos(phi) - r * np.sin(phi)])
        return f_

    def h(self, x, measurement, state):
        # measurement model y
        p = state.p
        q = state.q
        r = state.r

        Va = state.Va

        phi = x[0]
        theta = x[1]

        h_ = np.array([q * Va * np.sin(theta), r * Va * np.cos(theta) - p * Va * np.sin(theta) - \
                       MAV.gravity * np.cos(theta) * np.sin(phi), -q * Va * np.cos(theta) - \
                       MAV.gravity * np.cos(theta) * np.cos(phi)])
        return h_

    def propagate_model(self, measurement, state):
        for _ in range(self.N):
            # 1) propagate state
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)

            # 2) linearize
            A = jacobian(self.f, self.xhat, measurement, state)

            # 3) pull out updated angles
            phi   = self.xhat[0]
            theta = self.xhat[1]

            # 4) build gyro‐noise G on those updated angles
            G = np.array([
                [1,
                 np.sin(phi) * np.tan(theta),
                 np.cos(phi) * np.tan(theta)],
                [0,
                 np.cos(phi),
                 -np.sin(phi)]
            ])

            # 5) discretize and update P as before
            A_d     = np.eye(2) + A*self.Ts + 0.5*(A@A)*self.Ts**2
            G_d     = G * self.Ts
            Q_d     = self.Q * self.Ts**2
            Q_gyro_d = G_d @ self.Q_gyro @ G_d.T
            self.P  = A_d @ self.P @ A_d.T + Q_gyro_d + Q_d



    def measurement_update(self, measurement, state):
        # measurement model and jacobian
        h = self.h(self.xhat, measurement, state)  # shape (3,)
        C = jacobian(self.h, self.xhat, measurement, state)  # (3×2)
        # make y also 1-D
        y = np.array([measurement.accel_x,
                    measurement.accel_y,
                    measurement.accel_z])          # shape (3,)

        # innovation as 1-D
        res = y - h                                   # shape (3,)
        S_inv = np.linalg.inv(self.R_accel +
                            C @ self.P @ C.T)       # (3×3)

        # Mahalanobis distance is now a true scalar
        mah = res.T @ S_inv @ res                     # scalar

        if mah < self.gate_threshold:
            L = self.P @ C.T @ S_inv                  # (2×3)
            self.P = (np.eye(2) - L @ C) @ self.P @ (np.eye(2) - L @ C).T \
                    + L @ self.R_accel @ L.T
            self.xhat = self.xhat + L @ res           # note L @ res is (2,)



class EkfPosition:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    def __init__(self):
        self.Q = np.eye(7)
        self.R_gps = np.eye(4)
        self.R_pseudo = np.eye(2)
        self.N = 5  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat =  np.array([[0], [0], [30], [0], [0], [0], [0]])
        self.P = np.eye(7)
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999
        #self.pseudo_threshold = stats.chi2.isf() # AT - THIS LINE WAS GIVING PROBLEMS, SO I JUST MADE IT A CONSTANT. DOUBT CONSTANT WILL WORK LONG TERM.
        self.pseudo_threshold = 3
        self.gps_threshold = 100000 # don't gate GPS

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.north = self.xhat.item(0)
        state.east = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        Vg = x[2]
        chi = x[3]
        wn = x[4]
        we = x[5]
        psi = x[6]

        psidot = state.q * np.sin(state.phi) / np.cos(state.theta) + state.r + np.cos(state.phi) / np.cos(state.theta)
        phi = state.phi

        Vgdot = 1/Vg * (state.Va * psidot * (we * np.cos(psi) - wn * np.sin(psi)))

        f_ = np.array([Vg * np.cos(chi), Vg * np.sin(chi), Vgdot, MAV.gravity / Vg * np.tan(phi), 0, 0, psidot])
        return f_

    def h_gps(self, x, measurement, state):
        # measurement model for gps measurements
        pn = x[0]
        pe = x[1]
        Vg = x[2]
        chi = x[3]
        h_ = np.array([pn, pe, Vg, chi])
        return h_

    def h_pseudo(self, x, measurement, state):
        # measurement model for wind triangle pseudo measurement
        Vg = x[2]
        chi = x[3]
        wn = x[4]
        we = x[5]
        psi = x[6]

        h_ = np.array([state.Va * np.cos(psi) + wn - Vg * np.cos(chi), state.Va * np.sin(psi) + we - Vg * np.sin(chi)])
        return h_

    def propagate_model(self, measurement, state):
        # model propagation
        for i in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # convert to discrete time models
            I7 = np.eye(7)
            A_d = I7 + A * self.tTs + np.matmul(A, A) * 1/2 * self.Ts**2
            # update P with discrete time model
            self.P = np.matmul(np.matmul(A_d, self.P), A_d.T) + self.Ts ** 2 * self.Q

    def measurement_update(self, measurement, state):
        # always update based on wind triangle pseudu measurement
        h = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0, 0]]).T
        S_inv = np.linalg.inv(self.R_pseudo + np.matmul(np.matmul(C, self.P), C.T))
        if (y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
            L = np.matmul(np.matmul(self.P, C.T), S_inv)
            I7 = np.eye(7)
            self.P = np.matmul(np.matmul(I7 - np.matmul(L, C), self.P), (I7 - np.matmul(L, C))).T + \
                np.matmul(np.matmul(L, self.R_pseudo), L.T)
            self.xhat = self.xhat + np.matmul(L, y-h)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            S_inv = np.linalg.inv(self.R_gps + np.matmul(np.matmul(C, self.P), C.T))
            if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                L = np.matmul(np.matmul(self.P, C.T), S_inv)
                self.xhat = self.xhat + np.matmul(L, y-h)
                I7 = np.eye(7)
                self.P = np.matmul(np.matmul(I7 - np.matmul(L, C), self.P), (I7 - np.matmul(L, C))).T + \
                    np.matmul(np.matmul(L, self.R_gps), L.T)

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course


def jacobian(fun, x, measurement, state):
    f = fun(x, measurement, state)          # shape (m,)
    m = f.shape[0]
    n = x.shape[0]
    eps = 1e-4
    J = np.zeros((m, n))

    for i in range(n):
        x_eps = x.copy()
        x_eps[i] += eps
        f_eps = fun(x_eps, measurement, state)
        df    = (f_eps - f) / eps            # df.shape == (m,)
        J[:, i] = df                         # ← no [:,0]
    return J
