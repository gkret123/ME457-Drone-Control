
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")

import numpy as np
import parameters.control_parameters as CTRL
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from estimators.filters import AlphaFilter, ExtendedKalmanFilterContinuousDiscrete
import parameters.aerosonde_parameters as parameters



class Observer:
    def __init__(self, ts: float, initial_measurements: MsgSensors=MsgSensors()):
        self.Ts = ts  # sample rate of observer
        # initialized estimated state message
        self.estimated_state = MsgState()

        # low-pass filters for gyros and accelerometers
        # tune alpha values per section 8.3
        self.lpf_gyro_x = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)
        self.lpf_accel_x = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_x)
        self.lpf_accel_y = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_y)
        self.lpf_accel_z = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_z)
        # low-pass filters for pressures
        self.lpf_abs = AlphaFilter(alpha=0.7, y0=initial_measurements.abs_pressure)
        self.lpf_diff = AlphaFilter(alpha=0.7, y0=initial_measurements.diff_pressure)

        # EKF for attitude (phi, theta)
        self.attitude_ekf = ExtendedKalmanFilterContinuousDiscrete(
            f=self.f_attitude,
            Q=np.diag([1e-4, 1e-4]),           # process noise for phi, theta
            P0=np.diag([1e-2, 1e-2]),         # initial covariance
            xhat0=np.array([[0.0], [0.0]]),   # initial [phi, theta]
            Qu=np.diag([                    # input noise: p, q, r, Va
                SENSOR.gyro_sigma**2,
                SENSOR.gyro_sigma**2,
                SENSOR.gyro_sigma**2,
                SENSOR.abs_pres_sigma
            ]),
            Ts=ts,
            N=5
        )
        # EKF for position, heading, wind (pn, pe, Vg, chi, wn, we, psi)
        self.position_ekf = ExtendedKalmanFilterContinuousDiscrete(
            f=self.f_smooth,
            Q=np.diag([1e-1, 1e-1, 1e-2, 1e-2, 1e-6, 1e-6, 1e-3]),  # tune Q
            P0=np.diag([1.0, 1.0, 1.0, (5*np.pi/180)**2, 1.0, 1.0, (5*np.pi/180)**2]),
            xhat0=np.zeros((7,1)),
            Qu=np.diag([
                SENSOR.gyro_sigma**2,
                SENSOR.gyro_sigma**2,
                SENSOR.abs_pres_sigma,
                (np.radians(3))**2,
                (np.radians(3))**2
            ]),
            Ts=ts,
            N=10
        )
        # measurement noise
        self.R_accel = np.diag([
            SENSOR.accel_sigma**2,
            SENSOR.accel_sigma**2,
            SENSOR.accel_sigma**2
        ])
        # pseudo-measurement: enforce wind triangle
        self.R_pseudo = np.diag([1e-4, 1e-4])
        # GPS measurement noise
        self.R_gps = np.diag([
            SENSOR.gps_n_sigma**2,
            SENSOR.gps_e_sigma**2,
            SENSOR.gps_Vg_sigma**2,
            SENSOR.gps_course_sigma**2
        ])
        # store previous GPS to detect change
        self.gps_n_old = None
        self.gps_e_old = None
        self.gps_Vg_old = None
        self.gps_course_old = None

    def update(self, measurement: MsgSensors) -> MsgState:
        # 1) angular rates from low-pass filtered gyro minus bias 
        p = self.lpf_gyro_x.update(measurement.gyro_x) - self.estimated_state.bx
        q = self.lpf_gyro_y.update(measurement.gyro_y) - self.estimated_state.by
        r = self.lpf_gyro_z.update(measurement.gyro_z) - self.estimated_state.bz
        self.estimated_state.p = p
        self.estimated_state.q = q
        self.estimated_state.r = r
        # 2) altitude and airspeed from pressure sensors
        abs_p = self.lpf_abs.update(measurement.abs_pressure)
        diff_p = self.lpf_diff.update(measurement.diff_pressure)
        self.estimated_state.altitude = abs_p / (parameters.rho * parameters.gravity)
        self.estimated_state.Va = np.sqrt(2*diff_p/parameters.rho)
        # 3) attitude EKF
        u_att = np.array([[p], [q], [r], [self.estimated_state.Va]])
        # propagate
        xhat_att, _ = self.attitude_ekf.propagate_model(u_att)
        # measurement update
        y_acc = np.array([[measurement.accel_x], [measurement.accel_y], [measurement.accel_z]])
        xhat_att, _ = self.attitude_ekf.measurement_update(
            y=y_acc,
            u=u_att,
            h=self.h_accel,
            R=self.R_accel
        )
        self.estimated_state.phi = xhat_att[0,0]
        self.estimated_state.theta = xhat_att[1,0]
        # 4) position/heading/wind EKF
        u_pos = np.array([[q], [r], [self.estimated_state.Va], [self.estimated_state.phi], [self.estimated_state.theta]])
        # propagate
        xhat_pos, _ = self.position_ekf.propagate_model(u_pos)
        # pseudo measurement update
        y_pseudo = np.zeros((2,1))
        xhat_pos, _ = self.position_ekf.measurement_update(
            y=y_pseudo,
            u=u_pos,
            h=self.h_pseudo,
            R=self.R_pseudo
        )
        # GPS measurement update if new
        if measurement.gps_n != self.gps_n_old or measurement.gps_e != self.gps_e_old \
           or measurement.gps_Vg != self.gps_Vg_old or measurement.gps_course != self.gps_course_old:
            y_gps = np.array([
                [measurement.gps_n],
                [measurement.gps_e],
                [measurement.gps_Vg],
                [wrap(measurement.gps_course, xhat_pos[3,0])]
            ])
            xhat_pos, _ = self.position_ekf.measurement_update(
                y=y_gps,
                u=u_pos,
                h=self.h_gps,
                R=self.R_gps
            )
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course
        # assign state
        self.estimated_state.north = xhat_pos[0,0]
        self.estimated_state.east  = xhat_pos[1,0]
        self.estimated_state.Vg    = xhat_pos[2,0]
        self.estimated_state.chi   = xhat_pos[3,0]
        self.estimated_state.wn    = xhat_pos[4,0]
        self.estimated_state.we    = xhat_pos[5,0]
        self.estimated_state.psi   = xhat_pos[6,0]
        # remaining states
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta  = 0.0
        self.estimated_state.bx    = 0.0
        self.estimated_state.by    = 0.0
        self.estimated_state.bz    = 0.0
        return self.estimated_state

    def f_attitude(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # x = [phi, theta], u = [p, q, r, Va]
        phi, theta = x[0,0], x[1,0]
        p, q, r = u[0,0], u[1,0], u[2,0]
        phi_dot   = p + np.tan(theta)*(q*np.sin(phi) + r*np.cos(phi))
        theta_dot = q*np.cos(phi) - r*np.sin(phi)
        return np.array([[phi_dot], [theta_dot]])

    def h_accel(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # x = [phi, theta], u = [p, q, r, Va]
        phi, theta = x[0,0], x[1,0]
        g = parameters.gravity
        # gravity components in body frame
        return np.array([[ g*np.sin(theta) ],
                         [-g*np.cos(theta)*np.sin(phi)],
                         [-g*np.cos(theta)*np.cos(phi)]])

    def f_smooth(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # x = [pn, pe, Vg, chi, wn, we, psi]
        pn, pe, Vg, chi, wn, we, psi = x[:,0]
        q, r, Va, phi, theta = u[:,0]
        # kinematics
        pn_dot = Vg*np.cos(chi)
        pe_dot = Vg*np.sin(chi)
        # Vg_dot from wind triangle
        num = (Va*np.sin(psi)+we)*(Va*np.cos(psi)*r) + (Va*np.cos(psi)+wn)*(-Va*np.sin(psi)*r)
        Vg_dot = num/(Vg + 1e-6)
        # chi_dot
        chi_dot = ((Va*np.cos(psi)+wn)*(Va*np.cos(psi)*r) + (Va*np.sin(psi)+we)*(Va*np.sin(psi)*r)) / (Vg**2 + 1e-6)
        # wind assumed constant
        wn_dot = 0.0
        we_dot = 0.0
        # psi_dot from Euler kinematics
        psi_dot = (q*np.sin(phi) + r*np.cos(phi)) / np.cos(theta)
        return np.array([[pn_dot], [pe_dot], [Vg_dot], [chi_dot], [wn_dot], [we_dot], [psi_dot]])

    def h_pseudo(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # pseudo measurement for wind triangle
        pn, pe, Vg, chi, wn, we, psi = x[:,0]
        q, r, Va, phi, theta = u[:,0]
        # predicted body-frame ground velocities
        vbx = Va*np.cos(theta)*np.cos(psi)
        vby = Va*np.cos(theta)*np.sin(psi)
        # pseudo constraints: difference between measured ground vel and sum of body vel + wind = 0
        y1 = Vg*np.cos(chi) - (vbx + wn)
        y2 = Vg*np.sin(chi) - (vby + we)
        return np.array([[y1], [y2]])

    def h_gps(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # GPS measurement model: y = [pn, pe, Vg, chi]
        pn, pe, Vg, chi, wn, we, psi = x[:,0]
        return np.array([[pn], [pe], [Vg], [chi]])
