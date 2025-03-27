import numpy as np
import control as ct
import control.matlab as ctm
from matplotlib import pyplot as plt

# Parameters
M = 0.5
m = 0.2
b = 0.1
I = 0.006
g = 9.8
l = 0.3

# State space
den = I*(M+m)+M*m*l**2
A = np.array([
    [0,      1,              0,           0],
    [0, -(I+m*l**2)*b/den,  (m**2*g*l**2)/den,  0],
    [0,      0,              0,           1],
    [0, -(m*l*b)/den,       m*g*l*(M+m)/den,  0]
    ])
B = np.array([
    [0],
    [(I+m*l**2)/den],
    [0],
    [m*l/den]
    ])
C = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0]
    ])
C = np.eye(4)
D = np.array([
    [0],
    [0]
    ])
D = np.zeros((4, 1))

# print(A)

sys = ct.ss(A, B, C, D)
# print(sys)
# print(ct.poles(sys))

# print(ct.ctrb(sys.A, sys.B))
# print(np.linalg.matrix_rank(ct.ctrb(sys.A, sys.B)))

Q = np.diag([100, 1, 10, 0.1])  # x, theta, x_dot, theta_dot
R = 1

# K = ct.lqr(A, B, Q, R)
K = ct.lqr(A, B, Q, R)[0]
print(K.shape)
print(B.shape)

sys_cl = ct.ss(A-B@K, B, C, D)
y, t = ctm.step(sys_cl)
# y, t = ctm.initial(sys_cl, X0 = [0.5, 0, 0, 0])
y = np.squeeze(y)
# plt.plot(t, y, label = ("x", "theta"))  # , label=("x", "theta", "xdot", "thetadot")
plt.plot(t, y, label = ("x", "x_dot", "theta", "theta_dot"))  # , label=("x", "theta", "xdot", "thetadot")
U = np.squeeze(-K@y.T).T
plt.plot(t, U, label = "U")
plt.legend()
plt.grid()
plt.show()
