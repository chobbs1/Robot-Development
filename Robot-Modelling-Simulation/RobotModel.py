import numpy as np
from scipy.integrate import odeint
import math

class Robot():
    mass = 1
    I_G = 1
    battery = 100
    length = 0.5
    height = 0.5

    def __init__(self,IC):
        self.x = IC[0]
        self.x_dot = IC[1]
        self.y = IC[2]
        self.y_dot = IC[3]
        self.theta = IC[4]

    def printStatus(self):
        print("Robot Status:")
        print("x = {}".format(self.x))
        print("x_dot = {}".format(self.x_dot))
        print("y = {}".format(self.y))
        print("y_dot = {}".format(self.y_dot))
        print("theta = {}".format(self.theta))




    def updateDynamicModel(self,F_r,F_l):
        x = self.x
        x_dot = self.x_dot
        y = self.y
        y_dot = self.y_dot
        theta = self.theta
        theta_dot = self.theta_dot
        mass = self.mass
        I_G = self.I_G
        length = self.length
        height = self.height

        mLambda = (x_dot*theta_dot*math.cos(theta) + y_dot*theta_dot*math.sin(theta) + length*height/I_G*(F_r - F_l)) / (length**2/I_G+1/mass)

        x_dd = ((F_r+F_l)*math.cos(theta) - mLambda*math.sin(theta)) / mass
        y_dd = ((F_r+F_l)*math.sin(theta) + mLambda*math.cos(theta)) / mass
        theta_dd = (height*(F_r-F_l) - mLambda*length) / I_G

        eqns = [x_dd,y_dd,theta_dd]
        return eqns


    def updateControlInputs(self):
        return None





class MotorModel():
    x = 0
    def __init__(self,initial_conditions):
        self.x = initial_conditions[0]
