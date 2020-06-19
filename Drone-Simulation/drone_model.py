import math
import tkinter as tk
import numpy as np
from scipy.integrate import odeint

class Drone:
    # drone's physical parameters
    arm_length = 0.2
    arm_width = 0.02
    motor_height = 0.05
    motor_width = 0.02
    m = 0.5

    # environment variables
    g = 9.81
    SPEED_THRESHOLD = 20

    # other variables
    int_error = 0
    y_ref = 0

    def __init__(self,ICs,enVars):
        self.Z = ICs
        self.SCALE_FACTOR = enVars[0]
        self.HEIGHT_DISPLAY = enVars[1]

    def printStateVector(self):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        print("x = {}".format(x))
        print("x_dot = {}".format(x_dot))
        print("y = {}".format(y))
        print("y_dot = {}".format(y_dot))
        print("theta = {}".format(theta))
        print("theta_dot = {}".format(theta_dot))
        print("F_r = {}".format(self.F_r))
        print("F_l = {}".format(self.F_l))

    def draw_drone(self,displayCanvas):

        x = self.Z[0]
        y = self.Z[2]
        theta = self.Z[4]

        arm_length = self.arm_length
        arm_width = self.arm_width
        motor_height = self.motor_height
        motor_width = self.motor_width

        drone_unrotated_untranslated_vertices = [
            [-arm_length,0],
            [-arm_length,motor_height],
            [-arm_length+motor_width,motor_height],
            [-arm_length+motor_width,arm_width],
            [arm_length-motor_width,arm_width],
            [arm_length-motor_width,motor_height],
            [arm_length,motor_height],
            [arm_length,0],
        ]

        c_theta = math.cos(theta)
        s_theta = math.sin(theta)
        drone_rotated_translated_vertices = []

        for coords in drone_unrotated_untranslated_vertices:
            x0 = coords[0]
            y0 = coords[1]

            x1 = x0*c_theta - y0*s_theta
            y1 = x0*s_theta + y0*c_theta

            x1 = (x1+x)*self.SCALE_FACTOR
            y1 = self.HEIGHT_DISPLAY - (y1+y)*self.SCALE_FACTOR

            drone_rotated_translated_vertices.append([x1,y1])

        try:
            displayCanvas.delete(self.drone)
        except:
            print("kk")

        self.drone = displayCanvas.create_polygon(drone_rotated_translated_vertices, fill="black")

    def check_crash(self):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        if y<0:
            print("Crashed into groud")
            return False
        # elif x<0 or x>20:
        #     print("Off Screen")
        #     return True
        # elif x_dot>self.SPEED_THRESHOLD or y_dot>self.SPEED_THRESHOLD:
        #     print("Unrealistic speeds")
        #     return True

        return True

    def solve_dynamics(self,dt):

        t = arr = np.array([0, dt])

        y = self.Z[2]
        y_dot = self.Z[3]
        self.controller()

        # print(self.F_r)

        Z = [y,y_dot]

        Z = odeint(self.dynamics,Z,t)
        Z = Z[-1]

        self.Z[2] = Z[0]
        self.Z[3] = Z[1]
        # print(Z)
        # self.Z = Z[-1]

    def dynamics(self,Z,t):
        y_dot = self.Z[3]
        y_dd = ((self.F_r + self.F_l) - self.g)/self.m

        Z_dot = [y_dot,y_dd]
        return Z_dot

    # def dynamics(self,X,t):
    #     x = X[0]
    #     x_dot = X[1]
    #     y = X[2]
    #     y_dot = X[3]
    #     theta = X[4]
    #     theta_dot = X[5]
    #
    #     s_theta = math.sin(theta)
    #     c_theta = math.cos(theta)
    #
    #     x_dd = -(self.F_r + self.F_l)*s_theta/self.m
    #     y_dd = -((self.F_r + self.F_l)*c_theta - self.g)/self.m
    #     theta_dd = 12*(self.F_r + self.F_l)/(self.m*self.arm_length)
    #
        # Z_dot = [x_dot,x_dd,y_dot,y_dd,theta_dot,theta_dd]
        # return Z_dot

    # def dynamics(self,Z,t):
    #     x = self.Z[0]
    #     x_dot = self.Z[1]
    #     y = self.Z[2]
    #     y_dot = self.Z[3]
    #     theta = self.Z[4]
    #     theta_dot = self.Z[5]
    #
    #     s_theta = math.sin(theta)
    #     c_theta = math.cos(theta)
    #
    #     x_dd = -(self.F_r + self.F_l)*s_theta/self.m
    #     y_dd = -((self.F_r + self.F_l)*c_theta - self.g)/self.m
    #     theta_dd = 12*(self.F_r + self.F_l)/(self.m*self.arm_length)
    #
    #     Z_dot = [x_dot,x_dd,y_dot,y_dd,theta_dot,theta_dd]
    #     return Z_dot

    def controller(self):
        y_ref = self.y_ref

        y = self.Z[2]
        y_dot = self.Z[3]



        Kp = 20
        Kd = 10

        error = y_ref - y

        control_law = Kp*error - Kd*y_dot
        self.F_l = control_law/2
        self.F_r = control_law/2
