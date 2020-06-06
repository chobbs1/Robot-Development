import math
from RobotModel import Robot
import Visualisation

# define simulation environment conditions
duration = 100
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

mRobot.printStatus()

# for i in range(time_length):
#     physics_model.updateState(x,x_dot,y,y_dot,theta)
#     plot_robot.updatePlot(x,y,theta)

print("---Simulation Complete---")
