"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import os
import matplotlib.pyplot as plt
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import parameters.control_parameters as AP
from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from controllers.pi_control import PIControl
from controllers.pd_control_with_rate import PDControlWithRate
from controllers.tf_control import TFControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta


class Autopilot:
    def __init__(self, ts_control):
        # instantiate lateral-directional controllers
        self.roll_from_aileron = PDControlWithRate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.yaw_damper = TransferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)
        # self.yaw_damper = TFControl(
        #                 k=AP.yaw_damper_kr,
        #                 n0=0.0,
        #                 n1=1.0,
        #                 d0=AP.yaw_damper_p_wo,
        #                 d1=1,
        #                 Ts=ts_control)

        # instantiate longitudinal controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        self.altitude_from_pitch = PIControl(
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.airspeed_from_throttle = PIControl(
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0)
        self.commanded_state = MsgState()

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
    
    def update(self, cmd, state):
	
	#### TODO #####
        # lateral autopilot
        errorAirspeed = state.Va - cmd.airspeed_command
        chi_c = wrap(cmd.course_command, state.chi)
        errorCourse = self.saturate(state.chi - chi_c, -np.radians(15), np.radians(15))
        self.integratorCourse = self.integratorCourse + (self.Ts/2) * (errorCourse + self.errorCourseD1)
        self.errorCourseD1 = errorCourse
        xLat = np.array([[errorAirspeed * np.sin(state.beta)],  # v
                    [state.p],
                    [state.r],
                    [state.phi],
                    [errorCourse],
                    [self.integratorCourse]])
        
        tmp = -self.Klat @ xLat
        delta_a = self.saturate(tmp.item(0), -np.radians(30), np.radians(30))
        delta_r = self.saturate(tmp.item(1), -np.radians(30), np.radians(30))


        # longitudinal autopilot
        altitude_c = self.saturate(cmd.altitude_command, 
                          state.altitude - 0.2*AP.altitude_zone, 
                          state.altitude + 0.2*AP.altitude_zone)
    
        errorAltitude = state.altitude - altitude_c
        self.integratorAltitude = self.integratorAltitude \
                                + (self.Ts/2) * (errorAltitude + self.errorAltitudeD1)
        self.errorAltitudeD1 = errorAltitude
        self.integratorAirspeed = self.integratorAirspeed \
                                + (self.Ts/2) * (errorAirspeed + self.errorAirspeedD1)
        self.errorAirspeedD1 = errorAirspeed

        xLon = np.array([[errorAirspeed * np.cos(state.alpha)],  # u
                    [errorAirspeed * np.sin(state.alpha)],  # w
                    [state.q],
                    [state.theta],
                    [errorAltitude],
                    [self.integratorAltitude],
                    [self.integratorAirspeed]])

        tmp = -self.Klon @ xLon
        delta_e = self.saturate(tmp.item(0), -np.radians(30), np.radians(30))
        delta_t = self.saturate(tmp.item(1), 0.0, 1.0)

        # construct control outputs and commanded states
        delta = MsgDelta(elevator=0,
                         aileron=0,
                         rudder=0,
                         throttle=0)
        self.commanded_state.altitude = 0
        self.commanded_state.Va = 0
        self.commanded_state.phi = 0
        self.commanded_state.theta = 0
        self.commanded_state.chi = 0
        return delta, self.commanded_state

from models.mav_dynamics_control import MavDynamics
from message_types.msg_autopilot import MsgAutopilot

AP = Autopilot(0.01)
MAV = MavDynamics(0.01)
state = MsgState()
cmd = MsgAutopilot()
print(AP.update(cmd, state))