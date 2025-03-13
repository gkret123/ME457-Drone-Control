import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import numpy as np
from models.mav_dynamics_control import MavDynamics
# from models.compute_models import compute_ss_model, compute_tf_model, euler_state, quaternion_state, f_euler, df_dx, df_du, dT_dVa, dT_ddelta_t
import parameters.simulation_parameters as SIM
from message_types.msg_delta import MsgDelta
from models.trim import compute_trim

# initialize the mav
mav = MavDynamics(SIM.ts_simulation)

trim_state, trim_input = compute_trim(mav, 25., 5.*np.pi/180.)  # compute the trim conditions

pass