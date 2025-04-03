import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import numpy as np
from models.mav_dynamics_control import MavDynamics
# from models.compute_models import compute_ss_model, compute_tf_model, euler_state, quaternion_state, f_euler, df_dx, df_du, dT_dVa, dT_ddelta_t
from plotter.plot_results import plot_results
import parameters.simulation_parameters as SIM
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from message_types.msg_autopilot import MsgAutopilot
from models.trim import compute_trim
from tools.rotations import euler_to_quaternion, quaternion_to_euler
from controllers.autopilot import Autopilot
import plotter.plot_results as plot
import matplotlib.pyplot as plt

# Initialize the model, wind, and control inputs.
        # Simulation settings.
sim_time = 0.0
Ts = 0.01
sim_end_time = 10.0
mav = MavDynamics(Ts)
wind = np.array([[0.], [0.], [0.]])
delta = MsgDelta()


from models.mav_dynamics_control import MavDynamics
from message_types.msg_autopilot import MsgAutopilot

AutoP = Autopilot(0.01)
new_initial = euler_to_quaternion(0, 0, 0) # inital [phi, theta, psi] in radians
# throttle can only cope with inital angle of 0.07 rad
mav._state[6:10] = new_initial
mav._state[3] = 36  # initial airspeed in m/s
mav._update_true_state()
state = MsgState()
cmd = MsgAutopilot()


cmd.airspeed_command = 36  # commanded airspeed m/s
cmd.course_command = 0#np.pi/8  # commanded course angle in rad
cmd.altitude_command = -100  # commanded altitude in m
cmd.phi_feedforward = 0.3 #0.5  # feedforward command for roll angle


# Initialize history lists for time and state.
time_array = []
state_array = []
delta_array = []

# Simulation loop.
while sim_time < sim_end_time:
    # Update the model.
    state = mav.true_state
    delta, _ = AutoP.update(cmd, state)
    delta_array.append(delta)
    # delta.print()
    
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
plot.plot_results(time_array, state_array, title="Final Flight Dynamics", delta=delta_array)
