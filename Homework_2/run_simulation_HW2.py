import numpy as np
import matplotlib.pyplot as plt
import integrators_HW2 as intg

        
# Nonlinear state space form:
#  xdot = f(t, x, u), 
#   t: time
#   x: state vector
#   u: input

m = 1 
b = 0.25
k = 1

def f(t, x, u):
    return np.array([x[1], -(k/m)*x[0] - (b/m)*x[1] + u])

def step_input(t):
    return 1 if t >= 1 else 0  # Step applied at t = 1 unit

t = 0; 
x0 = np.array([1, 0]); # initial pos, vel
u = 0
dt = 0.1; 
n = 100

euler_integrator = intg.Euler(dt, f)
heun_integrator = intg.Heun(dt, f)
rk4_integrator = intg.RK4(dt, f)

t_history = [0]
x_e_history = [x0]
x_h_history = [x0]
x_rk4_history = [x0]

x_e, x_h, x_rk4 = x0, x0, x0

for i in range(n):
    x_e = euler_integrator.step(t, x_e, u)
    x_h = heun_integrator.step(t, x_h, u)
    x_rk4 = rk4_integrator.step(t, x_rk4, u)
    t = (i+1) * dt

    t_history.append(t)
    x_e_history.append(x_e)
    x_h_history.append(x_h)
    x_rk4_history.append(x_rk4)

# Analytical solution
omega = np.sqrt(k/m - (b/(2*m))**2)
t_history = np.array(t_history)
x_analytical = np.exp(-b/(2*m) * t_history) * np.cos(omega * t_history)
#v_analytical = np.diff(x_analytical)

intg.__doc__

#Position Plot
plt.figure(figsize=(10, 5))
plt.plot(t_history, [x_e[0] for x_e in x_e_history], label="Euler", linestyle="dashed")
plt.plot(t_history, [x_h[0] for x_h in x_h_history], label="Heun", linestyle="dotted")
plt.plot(t_history, [x_rk4[0] for x_rk4 in x_rk4_history], label="RK4", linestyle="dashdot")
plt.plot(t_history, x_analytical, label="Analytical", linestyle="solid", color='black')
plt.xlabel("Time")
plt.ylabel("Position")
plt.legend()
plt.title("Comparison of Euler, Heun, and Analytical Solution for Position")
plt.grid()
plt.show()

#Velocity Plot
plt.figure(figsize=(10, 5))
plt.plot(t_history, [x_e[1] for x_e in x_e_history], label="Euler", linestyle="dashed")
plt.plot(t_history, [x_h[1] for x_h in x_h_history], label="Heun", linestyle="dotted")
#plt.plot(t_history[1:], v_analytical, label="Analytical", linestyle="solid", color='black')
plt.xlabel("Time")
plt.ylabel("Velocity")
plt.legend()
plt.title("Comparison of Euler and Heun Solution for Velocity")
plt.grid()
plt.show()

#STEP Response
x_step = np.array([0.0, 0.0])  
x_step_h_history = [x_step]
x_step_e_history = [x_step]
t_step_history = [0]

step_integrator_h = intg.Heun(dt, f)

for i in range(n):
    t = (i + 1) * dt
    u_t = step_input(t)
    
    x_step_h = step_integrator_h.step(t, x_step, u_t)
    x_step = x_step_h
    
    t_step_history.append(t)
    x_step_h_history.append(x_step_h)
    
step_positions_h = [x[0] for x in x_step_h_history]

plt.plot(t_step_history, step_positions_h, label="Step Response -- Heun", linestyle="solid", color='blue')
plt.xlabel("Time")
plt.ylabel("Position")
plt.legend()
plt.title("Step Response @ t = 1")
plt.grid()
plt.show()