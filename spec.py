import numpy as np
from matplotlib import pyplot as plt


# ----------------------------------------------------------------------

# Constants
water_density = 1000  # kg/m3
water_viscosity = 1.002e-3
gravity = 9.81  # m/s2


# ----------------------------------------------------------------------

# Initial variables
# Input pump
pump_length = 3  # m
pump_diameter = 2  # cm
pump_power_max = 100  # W
# pump_power_min = 0  # W

# Tank 1
tank_1_height = 0.5  # m
tank_1_diameter = 0.75  # m

# Connecting pipe
connect_length = 4 / 100  # m
connect_diameter = 3.13 / 100  # m

# Tank 2
tank_2_height = 0.5  # m
tank_2_height_aim = 0.45  # m
tank_2_height_error = 1  # %
tank_2_diameter = 0.75  # m
tank_2_flow = 10  # mi

# Outflow pipe
outflow_length = 4 / 100  # m
outflow_diameter = 3.13 / 100  # m


# ----------------------------------------------------------------------

# Dependent variables
# Input pump
pump_area = np.pi * (pump_diameter / 200) ** 2  # m2
pump_rate = (pump_power_max) / (water_density * gravity * pump_length)  # m3/s
pump_velo = pump_rate / pump_area  # m/s

# Tank 1
tank_1_area = np.pi * (tank_1_diameter / 2) ** 2
tank_1_volume = tank_1_height * np.pi * (tank_1_diameter / 2) ** 2  # m3

# Connecting pipe
connect_area = np.pi * (connect_diameter / 200) ** 2  # m2
connect_resistance = (8 * water_viscosity * connect_length) / (np.pi * (connect_diameter / 2) ** 4)

# Tank 2
tank_2_height_error = np.array([tank_2_height_error,
                                tank_2_height_aim * tank_2_height_error / 100])  # [%, m]
tank_2_area = np.pi * (tank_2_diameter / 2) ** 2
tank_2_volume = tank_2_height * tank_2_area  # m3

# Outflow pipe
outflow_area = np.pi * (outflow_diameter / 2) ** 2  # m2
outflow_resistance = (8 * water_viscosity * outflow_length) / (np.pi * (outflow_diameter / 2) ** 4)


# ----------------------------------------------------------------------

# Functions
# Simulation function
def open_loop_simulation(height_initial, tank_area,
                         outflow_resistance, inflow_rate,
                         time_step):

    height_array = [height_initial]
    volume_array = [height_initial * tank_area]
    time_array = [0]
    outflow_array = [(gravity * height_array[-1]) / outflow_resistance]
    instance = 0
    volume_initial = volume_array[-1]
    if isinstance(inflow_rate, list):
        try:
            volume_in = inflow_rate[instance] * time_step
        except:
            volume_in = inflow_rate[-1] * time_step
    else:
        volume_in = inflow_rate * time_step
    outflow_rate = (gravity * height_array[-1]) / outflow_resistance
    volume_out = outflow_rate * time_step  # m3
    volume_final = volume_initial + volume_in - volume_out  # m3
    if volume_final < 0:
        volume_final = 0
    instance = instance + time_step
    height_array.append(volume_final / tank_area)
    volume_array.append(volume_final)
    outflow_array.append(outflow_rate)
    time_array.append(time_array[-1] + time_step)

    while instance<5 or time_array[-1]<600:
        volume_initial = volume_array[-1]
        if isinstance(inflow_rate, list):
            try:
                volume_in = inflow_rate[instance] * time_step
            except:
                volume_in = inflow_rate[-1] * time_step
        else:
            volume_in = inflow_rate * time_step
        # Hagen-Poiseuille equation
        outflow_rate = (gravity * height_array[-1]) / outflow_resistance
        # outflow_velo = np.sqrt(2 * gravity * height_array[-1])  # m/s
        # outflow_rate = outflow_velo * outflow_area  # m3/s
        volume_out = outflow_rate * time_step  # m3
        volume_final = volume_initial + volume_in - volume_out  # m3
        if volume_final < 0:
            volume_final = 0
        instance = instance + time_step
        height_array.append(volume_final / tank_area)
        volume_array.append(volume_final)
        outflow_array.append(outflow_rate)
        time_array.append(time_array[-1] + time_step)
        #print(height_array[-1], height_array[-2], volume_in)
    return height_array, volume_array, outflow_array, time_array


# Plotting function
def plt_global(x1_array, y1_array,
               x2_array, y2_array,
               x3_array, y3_array,
               x4_array, y4_array,
               x_label="",
               y1_label="", y2_label=""):

    fig = plt.figure(figsize=(10, 8))

    ax1 = fig.add_subplot(211)
    ax1.set_ylabel(y1_label, fontsize=15)
    ax1.tick_params(axis='x', labelbottom=False)
    ax1.tick_params(axis='y', labelsize=15)
    ax1.plot(x1_array, y1_array,
             '-', zorder=1, linewidth=1, label='Tank 1')
    ax1.plot(x2_array, y2_array,
             '-', zorder=1, linewidth=1, label='Tank 2')
    ax1.set_title("Filling", fontsize=15)
    ax1.legend(fontsize=15)

    ax2 = fig.add_subplot(212, sharex=ax1)
    ax2.set_ylabel(y2_label, fontsize=15)
    ax2.set_xlabel(x_label, fontsize=15)
    ax2.tick_params(axis='x', labelsize=15)
    ax2.tick_params(axis='y', labelsize=15)
    ax2.plot(x3_array, y3_array,
             '-', zorder=1, linewidth=1, label='Tank 1')
    ax2.plot(x4_array, y4_array,
             '-', zorder=1, linewidth=1, label='Tank 2')
    ax2.set_title("Emptying", fontsize=15)
    ax2.legend(fontsize=15)

    return fig, ax1, ax2

# ----------------------------------------------------------------------

# Simulation runs
# Filling tank 1
height_array, volume_array, outflow_array, time_array = open_loop_simulation(0, tank_1_area, connect_resistance, pump_rate * 0.8, 1)
print(f"Tank 1 reached final level of {height_array[-1]} meters in {time_array[-1]} seconds.")
x1_array, y1_array = time_array, height_array

# Filling tank 2
height_array, volume_array, outflow_array, time_array = open_loop_simulation(0, tank_1_area, connect_resistance, outflow_array, 1)
print(f"Tank 2 reached final level of {height_array[-1]} meters in {time_array[-1]} seconds.")
x2_array, y2_array = time_array, height_array

# Emptying tank 1
height_array, volume_array, outflow_array, time_array = open_loop_simulation(0.45, tank_1_area, connect_resistance, 0, 1)
print(f"Tank 1 reached final level of {height_array[-1]} meters in {time_array[-1]} seconds.")
x3_array, y3_array = time_array, height_array

# Emptying tank 2
height_array, volume_array, outflow_array, time_array = open_loop_simulation(y2_array[-1], tank_1_area, connect_resistance, outflow_array, 1)
print(f"Tank 2 reached final level of {height_array[-1]} meters in {time_array[-1]} seconds.")
x4_array, y4_array = time_array, height_array


plt_global(x1_array, y1_array,
           x2_array, y2_array,
           x3_array, y3_array,
           x4_array, y4_array,
           x_label="Time / s",
           y1_label="Height / m", y2_label="Height / m")











