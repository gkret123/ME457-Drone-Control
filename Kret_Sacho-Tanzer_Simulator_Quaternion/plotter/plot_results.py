import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_results(t, x, title="Flight Dynamics"):
    """
    Create static plots for the simulation results.
    t : list or np.array of time values
    x : list or np.array of state vectors, with each state being:
        [p_n, p_e, p_d, u, v, w, phi, theta, psi, p, q, r]
    """
    x = np.array(x)
    t = np.array(t)
    
    # Create 2D plots
    fig, axs = plt.subplots(2, 2, figsize=(10, 10))
    fig.suptitle(title)

    # Position in NED frame
    axs[0, 0].plot(t, x[:, 0], label="p_n (m)")
    axs[0, 0].plot(t, x[:, 1], label="p_e (m)")
    axs[0, 0].plot(t, x[:, 2], label="p_d (m)")
    axs[0, 0].legend()
    axs[0, 0].set_title("Position in NED Frame")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("Position (m)")
    axs[0, 0].grid(True)

    # Velocity in body frame
    axs[0, 1].plot(t, x[:, 3], label="u (m/s)")
    axs[0, 1].plot(t, x[:, 4], label="v (m/s)")
    axs[0, 1].plot(t, x[:, 5], label="w (m/s)")
    axs[0, 1].legend()
    axs[0, 1].set_title("Linear Velocity in Body Frame")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Velocity (m/s)")
    axs[0, 1].grid(True)

    # Euler angles
    axs[1, 0].plot(t, x[:, 6], label="phi (rad)")
    axs[1, 0].plot(t, x[:, 7], label="theta (rad)")
    axs[1, 0].plot(t, x[:, 8], label="psi (rad)")
    axs[1, 0].legend()
    axs[1, 0].set_title("Euler Angles")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Angle (rad)")
    axs[1, 0].grid(True)

    # Angular velocities
    axs[1, 1].plot(t, x[:, 9], label="p (rad/s)")
    axs[1, 1].plot(t, x[:, 10], label="q (rad/s)")
    axs[1, 1].plot(t, x[:, 11], label="r (rad/s)")
    axs[1, 1].legend()
    axs[1, 1].set_title("Angular Velocities")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("Angular Velocity (rad/s)")
    axs[1, 1].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Create 3D plot for trajectory
    fig3d = plt.figure(figsize=(10, 6))
    ax3d = fig3d.add_subplot(projection='3d')
    ax3d.plot(x[:, 0], x[:, 1], x[:, 2], label='Trajectory')
    ax3d.scatter(x[0, 0], x[0, 1], x[0, 2], marker='o', color='g', s=50, label="Start")
    ax3d.scatter(x[-1, 0], x[-1, 1], x[-1, 2], marker='o', color='r', s=50, label="End")
    ax3d.set_xlabel("North (m)")
    ax3d.set_ylabel("East (m)")
    ax3d.set_zlabel("Down (m)")
    ax3d.legend()
    plt.suptitle("3D Position")
    
    # Display the figures
    plt.show()
