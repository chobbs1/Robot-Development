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
from scipy.integrate import odeint

def getlogTitle():
    today = date.today()
    current_date = today.strftime("_%b_%d_%Y")
    t = timeModule.localtime()
    current_time = timeModule.strftime("%H-%M-%S", t)
    log_title = "Data_Log_" + current_time + current_date
    return log_title

def beginDataLogging():
    try:
        file = open("Logs/"+getlogTitle()+".csv", 'w',newline='')
    except FileNotFoundError:
        os.mkdir("Logs")
        file = open("Logs/"+getlogTitle()+".csv", 'w',newline='')
    dataLogger = csv.writer(file,delimiter=',')
    dataLogger.writerow(["x","x_dot","y","y_dot","theta"])
    return dataLogger,file


#------------------------------------------------------------------------
# run simulation
print("---Begin Simulation---")
# mRobot = Robot(initial_conditions)
mPlotter = VisualisationTools()
# dataLogger,file = beginDataLogging()

# define simulation environment conditions
duration = 100
dt = 0.01
t0 = 0

mu = 0.3
m = 1
I_G = 1
g = 9.81
battery = 100
length = 0.5
height = 0.5

def controller(t):
    if t<2:
        F_r = 5
        F_l = 5
    elif(t<4):
        F_r = 5
        F_l = 0
    else:
        F_r = 0
        F_l = 0
    control_law = [F_r,F_l]
    return control_law

def forceModel(Z,control_law):
    x_dot = Z[3]
    y_dot = Z[4]
    theta_dot = Z[5]

    F_r_motor = control_law[0]
    F_l_motor = control_law[1]

    if x_dot>0 or y_dot>0 or theta_dot>0:
        F_r_fric = mu*m*g/2
        F_l_fric = mu*m*g/2
    else:
        F_r_fric = 0
        F_l_fric = 0

    F_r = F_r_motor - F_r_fric
    F_l = F_l_motor - F_l_fric

    return F_r,F_l


def dynamics(Z,t):
    x = Z[0]
    y = Z[1]
    theta = Z[2]
    x_dot = Z[3]
    y_dot = Z[4]
    theta_dot = Z[5]

    control_law = controller(t)
    F_r,F_l = forceModel(Z,control_law)

    Lambda = (x_dot*theta_dot*math.cos(theta) + y_dot*theta_dot*math.sin(theta) + length*height/I_G*(F_r - F_l)) / (length**2/I_G+1/m)

    x_dd = ((F_r+F_l)*math.cos(theta) - Lambda*math.sin(theta)) / m
    y_dd = ((F_r+F_l)*math.sin(theta) + Lambda*math.cos(theta)) / m
    theta_dd = (height*(F_r-F_l) - Lambda*length) / I_G

    Z_dot = [x_dot,y_dot,theta_dot,x_dd,y_dd,theta_dd]
    return Z_dot

ICs = [0,0,0,0,0,0]
t = np.arange(t0,duration,dt)
Z = odeint(dynamics,ICs,t)

x = Z[:,0]
y = Z[:,1]
theta = Z[:,2]
x_dot = Z[:,3]
y_dot = Z[:,4]
theta_dot = Z[:,5]


###------------visualise output-------------###
# mPlotter.animateRobot(logger_x,logger_y,logger_theta,logger_t)
mPlotter.plot_state_vs_time(t,Z)
# mPlotter.plot_vs_time(t,y_dot,title="Y_dot vs. Time",ylabel="Vel (m/s)")

# print(x)
# file.close()
print("---Simulation Complete---")





# m = 1
# g = 9.81
# ICs = [0,0]
#
# error_log = []
# error_int_log = []
# time_log = []

# def trap_int(e0,e1):
#     return (e0 + e1)*dt/2


# def controller(X,t):
#     y_ref = 2
#
#     Kp = 5
#     Ki = 0.001
#     Kd = 2
#
#     y = X[0]
#     y_dot = X[1]
#
#     error = y_ref - y
#     if t == 0:
#         global e0
#         global int_error
#         int_error = 0.00000001
#     else:
#         int_error += trap_int(e0,error)
#     e0 = error
#     print(int_error)
#
#
#     error_log.append(error)
#     error_int_log.append(int_error)
#     time_log.append(t)
#
#     control_law = Kp*error + Ki*int_error - Kd*y_dot
#     # print(control_law)
#
#
#     return control_law

# def dynamics(X,t):
#     y = X[0]
#     y_dot = X[1]
#
#     Uy = controller(X,t)
#     y_dd = Uy/m- g
#
#     X_dot = [y_dot,y_dd]
#
#     return X_dot
