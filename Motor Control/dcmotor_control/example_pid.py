import numpy as np
import parameters as P
from integrators import get_integrator
from pid import PIDControl
from matplotlib import pyplot as plt


class Controller:
    def __init__(self):
        self.pid = PIDControl(P.kp, P.ki, P.kd, P.umax, P.sigma, P.Ts)
        
    def update(self, r, y):
        return self.pid.PID(r, y)
    
class System:
    def __init__(self, x0):
        self.integrator = get_integrator(P.Ts, self.f, integrator="RK4")
        self.x = x0

    def f(self, t, x, u):
        return (P.K*u - x)/P.tau

    def update(self, u):
        self.x = self.integrator.step(0, self.x, u)
        return self.x

# Init system and feedback controller
system = System(0)
controller = Controller()

# Simulate step response
t_history = [0]
y_history = [0]
u_history = [0]

r = 0
y = 0
t = 0
r_history = [r]
for i in range(P.nsteps):
    r = np.sin(i/10)
    r_history.append(r)
    u = controller.update(r, y) 
    y = system.update(u) 
    t += P.Ts

    t_history.append(t)
    y_history.append(y)
    u_history.append(u)

# Plot response y due to step change in r
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Plot response y due to step change in r
ax1.plot(t_history, y_history, label = "y")
ax1.plot(t_history, r_history, label = "r")
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Response')
ax1.set_title('Response due to step change in setpoint')
ax1.grid()

# Plot actuation signal
ax2.plot(t_history, u_history)
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Actuation Signal')
ax2.set_title('Actuation Signal')
ax2.grid()

plt.legend()
plt.tight_layout()
plt.show()