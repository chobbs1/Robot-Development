import math
import numpy as np
from RobotModel import Robot
from Visualisation import VisualisationTools
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from datetime import date,time
import time as timeModule
import csv
import os

def getlogTitle():
    today = date.today()
    current_date = today.strftime("_%b_%d_%Y")
    t = timeModule.localtime()
    current_time = timeModule.strftime("%H-%M-%S", t)
    log_title = "Data_Log_" + current_time + current_date
    return log_title

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

def beginDataLogging():
    try:
        file = open("Logs/"+getlogTitle()+".csv", 'w',newline='')
    except FileNotFoundError:
        os.mkdir("Logs")
        file = open("Logs/"+getlogTitle()+".csv", 'w',newline='')
    dataLogger = csv.writer(file,delimiter=',')
    dataLogger.writerow(["x","x_dot","y","y_dot","theta"])
    return dataLogger,file


dataLogger,file = beginDataLogging()



for i in range(int(duration/dt)):
    time += dt
    x = i


    
    dataLogger.writerow([x,x_dot,y,y_dot,theta])



###---------------Log Output----------------###


###------------visualise output-------------###
# mPlotter.animateRobot(x,y,t)
# mPlotter.plot_vs_time(t,y,title="Y vs. Time",ylabel="Acc (m/s)")



file.close()
print("---Simulation Complete---")
