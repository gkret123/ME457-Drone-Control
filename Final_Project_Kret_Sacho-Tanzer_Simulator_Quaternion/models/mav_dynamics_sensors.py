"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")

import numpy as np
from message_types.msg_sensors import MsgSensors
import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from models.mav_dynamics_control import MavDynamics as MavDynamicsNoSensors
from tools.rotations import quaternion_to_rotation, quaternion_to_euler, euler_to_rotation

class MavDynamics(MavDynamicsNoSensors):
    def __init__(self, Ts):
        super().__init__(Ts)
        # initialize the sensors message
        self._sensors = MsgSensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.

        # NOTE: we made the next variables, maybes hsould be done differently
        self._nu_n_prev = 0.  # previous value of nu_n
        self._nu_e_prev = 0.  # previous value of nu_e
        self._nu_h_prev = 0.  # previous value of nu_h

        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.

    def sensors(self):
        "Return value of sensors on MAV: gyros, accels, absolute_pressure, dynamic_pressure, GPS"
       
        # simulate rate gyros(units are rad / sec)
        self._sensors.gyro_x = self._state.item(10) + SENSOR.gyro_x_bias + np.random.normal(0, SENSOR.gyro_sigma)
        self._sensors.gyro_y = self._state.item(11) + SENSOR.gyro_y_bias + np.random.normal(0, SENSOR.gyro_sigma)
        self._sensors.gyro_z = self._state.item(12) + SENSOR.gyro_z_bias + np.random.normal(0, SENSOR.gyro_sigma)

        #state = [pn, pe, h, u, v, w, q0, q1, q2, q3, p, q, r]
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        # simulate accelerometers(units of g)

        #accel_x = u_dot +qw-rv+gsin(theta)
        self._sensors.accel_x = self._state.item(3) + self._state.item(11) * self._state.item(5) - self._state.item(12) * self._state.item(4) + MAV.gravity * np.sin(theta)
        #accel_y = v_dot + ru-pw-gcos(theta)sin(phi)
        self._sensors.accel_y = self._state.item(4) + self._state.item(12) * self._state.item(3) - self._state.item(10) * self._state.item(5) - MAV.gravity * np.cos(theta) * np.sin(phi)
        #accel_z = w_dot + pv-qu-gcos(theta)cos(phi)
        self._sensors.accel_z = self._state.item(5) + self._state.item(10) * self._state.item(4) - self._state.item(11) * self._state.item(3) - MAV.gravity * np.cos(theta) * np.cos(phi)

        # simulate magnetometers
        # magnetic field in provo has magnetic declination of 12.5 degrees
        # and magnetic inclination of 66 degrees
        M = 1  # overall strength of magnetic field. NOTE: I made this up
        inclination = np.radians(66)  # magnetic inclination
        declination = np.radians(12.5)  # magnetic declination
        self._sensors.mag_x = M*np.cos(psi-declination)*np.cos(inclination) + np.random.normal(0, SENSOR.mag_sigma)  # magnetic field in NED frame
        self._sensors.mag_y = -M*np.sin(psi-declination)*np.cos(inclination) + np.random.normal(0, SENSOR.mag_sigma)  # magnetic field in NED frame 
        self._sensors.mag_z = M*np.sin(inclination) + np.random.normal(0, SENSOR.mag_sigma)  # magnetic field in NED frame

        # simulate pressure sensors 

        # NOTE: not 100% sure about these either: (they come from inverse of formulas in observer.py)
        self._sensors.abs_pressure = -self._state.item(2) * MAV.rho * MAV.gravity + np.random.normal(0, SENSOR.abs_pres_sigma)  # absolute pressure in Pascals
        self._sensors.diff_pressure = self.true_state.Va**2 * MAV.rho / 2. + np.random.normal(0, SENSOR.diff_pres_sigma)  # differential pressure in Pascals
        
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n = np.random.normal(0, SENSOR.gps_n_sigma)  # GPS noise in north direction
            self._gps_eta_e = np.random.normal(0, SENSOR.gps_e_sigma)  # GPS noise in east direction
            self._gps_eta_h = np.random.normal(0, SENSOR.gps_h_sigma)  # GPS noise in altitude direction
            
            self._nu_n_prev = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._nu_n_prev + self._gps_eta_n
            self._nu_e_prev = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._nu_e_prev + self._gps_eta_e
            self._nu_h_prev = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._nu_h_prev + self._gps_eta_h
            
            self._sensors.gps_n = self._state.item(0) + self._nu_n_prev
            self._sensors.gps_e = self._state.item(1) + self._nu_e_prev
            self._sensors.gps_h = -self._state.item(2) + self._nu_h_prev
            self._sensors.gps_Vg = np.sqrt(self._state.item(3)**2 + self._state.item(4)**2 + self._state.item(5)**2) + np.random.normal(0, SENSOR.gps_Vg_sigma)  # NOTE: these formulas are copilot, make sure they're correct
            self._sensors.gps_course = np.arctan2(self._state.item(4), self._state.item(3)) + np.random.normal(0, SENSOR.gps_course_sigma)
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation

        return self._sensors

    def external_set_state(self, new_state):
        self._state = new_state

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
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
        self.true_state.bx = SENSOR.gyro_x_bias
        self.true_state.by = SENSOR.gyro_y_bias
        self.true_state.bz = SENSOR.gyro_z_bias
        self.true_state.camera_az = self._state.item(13)
        self.true_state.camera_el = self._state.item(14)