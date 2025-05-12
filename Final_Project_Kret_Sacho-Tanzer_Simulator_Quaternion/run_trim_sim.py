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
from tools.rotations import euler_to_quaternion
from controllers.autopilot import Autopilot
import plotter.plot_results as plot

# Initialize the model, wind, and control inputs.
        # Simulation settings.
sim_time = 0.0
Ts = 0.01
sim_end_time = 10.0
mav = MavDynamics(Ts)
wind = np.array([[0.], [0.], [0.]])
delta = MsgDelta()

mav_temp = MavDynamics(Ts)
trim_state, trim_input = compute_trim(mav_temp, 62.8, 0.0)
print("Trim State:")
print(trim_state)
print("Trim Input:")
trim_input.print()

mav_temp = MavDynamics(Ts)
mav_temp._state = trim_state
mav_temp._update_true_state()
print("Trimmed State derivative:")
print(mav_temp._f(trim_state, mav_temp._forces_moments(trim_input)))

mav._state = trim_state

# Initialize history lists for time and state.
time_array = []
state_array = []
    
# Simulation loop.
while sim_time < sim_end_time:
    mav.update(trim_input, wind)
    
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