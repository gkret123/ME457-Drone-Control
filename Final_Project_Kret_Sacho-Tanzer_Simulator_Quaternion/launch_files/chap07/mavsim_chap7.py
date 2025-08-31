"""
mavsim_python
    - Chapter 7 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/18/2020 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
import numpy as np
import matplotlib.pyplot as plt
import parameters.simulation_parameters as SIM
from tools.signals import Signals
from models.mav_dynamics_sensors import MavDynamics
from models.wind_simulation import WindSimulation
from controllers.autopilot import Autopilot
from viewers.view_manager import ViewManager
from message_types.msg_path import MsgPath
import time
from parameters import control_parameters as CP


# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
viewers = ViewManager(path=True, 
                      data=True,
                      sensors=True,
                      animation=True,
                      video=False, video_name='chap7.mp4')
path = MsgPath()
# autopilot commands
from message_types.msg_autopilot import MsgAutopilot
commands = MsgAutopilot()
Va_command = Signals(dc_offset=65.0,
                     amplitude=0.0,
                     start_time=.0,
                     frequency=0.01)
altitude_command = Signals(dc_offset=500.0,
                           amplitude=10.0,
                           start_time=10.0,
                           frequency=0.02)
course_command = Signals(dc_offset=np.radians(45),
                         amplitude=np.radians(0),
                         start_time=5.0,
                         frequency=0.015)

# initialize the simulation time
sim_time = SIM.start_time
end_time = 400

true_state_list = []
commanded_state_list = []
euler_true_list = []
euler_commanded_list = []
north_list = []
east_list = []
altitude_list = []
course_list = []

stage = "Accelerate"
print(f"{stage=}")
timer = 0
timestamp = sim_time

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:

    # -------autopilot commands-------------
    if stage == "Accelerate":
        commands.airspeed_command = min(62.8, 40 + 0.5 * (sim_time - timestamp))
        commands.course_command = np.radians(0.0)
        commands.altitude_command = 0.1
        if mav.true_state.Va >= 28:
            stage = "Takeoff"
            print(f"{stage=}")
            timestamp = sim_time
    elif stage == "Takeoff":
        commands.airspeed_command = min(62.8, 40 + 0.5 * (sim_time - timestamp))
        commands.course_command = np.radians(0.01)
        commands.altitude_command = min(200, 0.1 + 3 * (sim_time - timestamp))
        if (mav.true_state.altitude >= 198) and (mav.true_state.altitude <= 202) and (abs(mav._state[5]) <= 0.5):
            stage = "Cruise"
            print(f"{stage=}")
            timer = 5
            timestamp = sim_time
    elif stage == "Cruise":
        timer -= SIM.ts_simulation
        commands.airspeed_command = 62.8
        commands.course_command = np.radians(0.1)
        commands.altitude_command = 200
        if timer <= 0:
            stage = "Turn"
            CP.zeta_course = 0.6
            print(f"{stage=}")
            timestamp = sim_time
    elif stage == "Turn":
        commands.airspeed_command = 62.8
        commands.course_command = min(np.radians(140.0), np.radians(0.0) +  0.025 * (sim_time - timestamp))
        commands.altitude_command = 200.0
        if (mav.true_state.chi >= np.radians(89.0)) and (mav.true_state.chi <= np.radians(91.0)):
            stage = "Land"
            CP.zeta_course = 5
            CP.zeta_pitch = 1.5
            print(f"{stage=}")
            timestamp = sim_time
    elif stage == "Land":
        commands.airspeed_command = 62.8 - 1 * (sim_time - timestamp)
        commands.course_command = np.radians(90.)
        commands.altitude_command = 200 - 1.25 * (sim_time - timestamp)
        if (mav.true_state.altitude <= 5.0) and (mav.true_state.altitude >= 0.0) and (abs(mav._state[5]) <= 2.0):
            stage = "touchdown"
            print(f"{stage=}")
            viewers.close(dataplot_name="ch8_data_plot", 
              sensorplot_name="ch8_sensor_plot")
            break
            

    # -------autopilot-------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = mav.true_state  # uses true states in the control
    delta, commanded_state = autopilot.update(commands, estimated_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics
    if mav.true_state.altitude <= 0.0:
       mav.true_state.altitude = 0.01
        # mav.true_state.phi = 0.01
        # mav.true_state.psi = 0.01
       mav._state[2] = 0.01
    # -------update viewer-------------
    viewers.update(
        sim_time,
        true_state=mav.true_state,  # true states
        commanded_state=commanded_state,  # commanded states
        delta=delta, # inputs to MAV
        measurements=measurements,  # measurements
        path=path, # path
    )
    true_state_list.append(mav.true_state.altitude)
    commanded_state_list.append(commanded_state.altitude)
    euler_true_list.append([mav.true_state.phi, mav.true_state.theta, mav.true_state.psi])
    euler_commanded_list.append([commanded_state.phi, commanded_state.theta, commanded_state.psi])
    north_list.append([mav.true_state.north, commanded_state.north])
    east_list.append([mav.true_state.east, commanded_state.east])
    altitude_list.append([mav.true_state.altitude, commanded_state.altitude])
    course_list.append([mav.true_state.chi, commanded_state.chi])
    # # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation
    time.sleep(0.002) # slow down the simulation for visualization
    
viewers.close(dataplot_name="ch7_data_plot", 
              sensorplot_name="ch7_sensor_plot")

plt.figure()
plt.plot(true_state_list, label='True State')
plt.plot(commanded_state_list, label='Commanded State')
plt.title('Altitude')
plt.xlabel('Time')
plt.ylabel('Altitude (m)')
plt.legend()


plt.figure()
plt.plot(euler_true_list, label=['\u03D5 True State', '\u03B8 True State', '\u03C8 True State'])
plt.plot(euler_commanded_list, label=['\u03D5 Commanded State', '\u03B8 Commanded State', '\u03C8 Commanded State'])
plt.title('Euler Angles')
plt.xlabel('Time')
plt.ylabel('Euler Angles (rad)')
plt.legend()


#plot true state position as a 3dplot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract trajectory data properly
true_north = [x[0] for x in north_list]
true_east = [x[0] for x in east_list]
true_alt = [x[0] for x in altitude_list]

com_north = [x[1] for x in north_list]
com_east = [x[1] for x in east_list]
com_alt = [x[1] for x in altitude_list]

# Plot complete trajectories
ax.plot(true_north, true_east, true_alt, label='True State Position')
#ax.plot(com_north, com_east, com_alt, label='Commanded State Position')

# Mark start and end points
ax.scatter(true_north[0], true_east[0], true_alt[0], color='red', label='Start Position')
ax.scatter(true_north[-1], true_east[-1], true_alt[-1], color='green', label='End Position')

ax.legend()
ax.set_xlabel('North Position (m)')
ax.set_ylabel('East Position (m)')
ax.set_zlabel('Altitude (m)')
ax.set_title('MAV Position')

#plot course angle in degrees
plt.figure()
plt.plot([np.degrees(x[0]) for x in course_list], label='True State')
plt.plot([np.degrees(x[1]) for x in course_list], label='Commanded State')

plt.title('Course Angle')
plt.xlabel('Time')
plt.ylabel('Course Angle (rad)')
plt.legend()
plt.show()