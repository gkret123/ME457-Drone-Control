import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_results(t, x, title = "Test Case"):
    x = np.array(x)
    fig, axs = plt.subplots(2, 2, figsize=(10, 10))
    plt.suptitle(title)
    # Position in NED frame
    #An increasing p_d represents a decreasing altitude
    axs[0,0].plot(t, x[:, 0:3])
    axs[0,0].legend(["p_n (m)", "p_e (m)", "p_d (m)"])
    axs[0,0].set_title("Position in NED Frame")
    axs[0,0].set_xlabel("Time (s)")
    axs[0,0].set_ylabel("Position (m)")

    # Velocity in body frame
    axs[0, 1].plot(t, x[:, 3:6])
    axs[0,1].legend(["u (m/s)", "v (m/s)", "w (m/s)"])
    axs[0,1].set_title("Linear Velocity in Body Frame")
    axs[0,1].set_xlabel("Time (s)")
    axs[0,1].set_ylabel("Velocity (m/s)")

    # Euler angles
    axs[1,0].plot(t, x[:, 6:9])
    axs[1,0].legend(["phi (rad)", "theta (rad)", "psi (rad)"])
    axs[1,0].set_title("Euler Angles")
    axs[1,0].set_xlabel("Time (s)")
    axs[1,0].set_ylabel("Angle (rad)")

    # Angular velocities
    axs[1,1].plot(t, x[:, 9:12])
    axs[1,1].legend(["p (rad/s)", "q (rad/s)", "r (rad/s)"])
    axs[1,1].set_title("Angular Velocities")
    axs[1,1].set_xlabel("Time (s)")
    axs[1,1].set_ylabel("Angular Velocity (rad/s)")

    plt.tight_layout()


    ax = plt.figure(figsize=(10,6)).add_subplot(projection='3d')
    # Prepare arrays x, y, z
    
    ax.plot(*x[:, 0:3].T, label='Position Vs. Time')
    ax.set_xlabel("North")
    ax.set_ylabel("East")
    ax.set_zlabel("Down")
    plt.show()