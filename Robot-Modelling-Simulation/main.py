import math
import numpy as np
from RobotModel import Robot
from Visualisation import VisualisationTools
from matplotlib import pyplot as plt
import matplotlib.animation as animation

# define simulation environment conditions
duration = 10
time = 0
dt = 0.01

# define robot initial conditions
x = 0
x_dot = 0
y = 0
y_dot = 0
theta = 0

# run simulation
print("---Begin Simulation---")
initial_conditions = [x,x_dot,y,y_dot,theta]
mRobot = Robot(initial_conditions)
mPlotter = VisualisationTools()


# for i in range(int(duration/dt)):
#     time += dt
t = np.arange(0,duration,dt)
y = np.sin(2*np.pi*t)*np.exp(-t)
x = 5*np.ones(len(y))


###------------visualise output-------------###
# mPlotter.animateRobot(x,y,t)
# mPlotter.plot_vs_time(t,y,title="Y vs. Time",ylabel="Acc (m/s)")




print("---Simulation Complete---")
