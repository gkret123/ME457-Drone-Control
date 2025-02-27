from scipy.optimize import minimize
import numpy as np


def trim_conditions(aircraft, wind, deflections, V_a, gamma, R):
    def trim_objective(x):
        state = np.array([0, 0, -R, V_a, 0, 0, 0, 0, 0, x[0], x[1], x[2]])
        aircraft.set_state(state)
        aircraft.set_wind(wind)
        aircraft.set_deflections(deflections)
        aircraft.set_gamma(gamma)
        aircraft.set_R(R)
        aircraft.update_state()
        return np.linalg.norm(aircraft.state[3:6] - np.array([V_a, 0, 0]))**2
