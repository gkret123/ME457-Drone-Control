    # run the simulation, repeatedly updating and appending results
    while sim_time < sim_end_time:
        time_array.append(sim_time)
        state_array.append(copy.deepcopy(mav.true_state))
        control_array.append(delta)
        wind_array.append(wind)
        wind_array.append(wind)
        sim_time += Ts
