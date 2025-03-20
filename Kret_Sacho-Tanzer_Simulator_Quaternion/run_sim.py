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
from models.trim import compute_trim
from tools.rotations import euler_to_quaternion


# Initialize the model, wind, and control inputs.
    # Simulation settings.

sim_time = 0.0
Ts = 0.01
sim_end_time = 10.0
mav = MavDynamics(Ts)
wind = np.array([[0.], [0.], [0.], [0], [0], [0]])
# delta = trim_input

# Initialize history lists for time and state.
time_array = []
state_array = []


# initialize a temporary mav to calculate trim
mav_temp = MavDynamics(SIM.ts_simulation)


Va  = 30
gamma = np.deg2rad(0)

trim_state, trim_input = compute_trim(mav_temp, Va, gamma)  # compute the trim conditions

mav_temp = MavDynamics(SIM.ts_simulation)

print(mav_temp._f(mav_temp._state, mav_temp._forces_moments(trim_input)))  # shows issue: pdot != 0 (accelerating in roll)
trim_input.print()
# quit()

delta = trim_input

e = euler_to_quaternion(0., gamma, 0.)
state0 = np.array([[0],  # pn
            [0],  # pe
            [0],  # pd
            [Va],  # u
            [0.], # v
            [0.], # w
            [e[0,0]],  # e0
            [e[1,0]],  # e1
            [e[2,0]],  # e2
            [e[3,0]],  # e3
            [0.], # p
            [0.], # q
            [0.]  # r
            ])

mav._state = state0

# Simulation loop.
while sim_time < sim_end_time:
    # print(f"Elevator: {delta.elevator} Aileron: {delta.aileron} Rudder: {delta.rudder} Throttle: {delta.throttle}")
    # if sim_time.is_integer() and sim_time % 10 == 0:  # every 10 seconds update control inputs
        # mav_temp = MavDynamics(SIM.ts_simulation)
        
    
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
plot_results(time_array, state_array, title="Final Flight Dynamics")

