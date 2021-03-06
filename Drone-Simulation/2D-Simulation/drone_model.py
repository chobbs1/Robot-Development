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

    F_l = 0
    F_r = 0

    x_ref = 0
    y_ref = 0

    # gains
    Kp_x = 0
    Kd_x = 0
    Kp_y = 0
    Kd_y = 0
    Kp_theta = 0
    Kd_theta = 0

    def __init__(self,ICs,enVars):
        self.Z = ICs
        self.SCALE_FACTOR = enVars[0]
        self.HEIGHT_DISPLAY = enVars[1]

    def delete_drone(self,displayCanvas):
        displayCanvas.delete(self.drone)

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

        # x_target
        # y_target

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
            # displayCanvas.delete(self.target)
        except AttributeError:
            pass
        r = 5
        # self.target = displayCanvas.create_oval(self.x_ref-r, self.y_ref-r, self.x_ref+r, self.y_ref+r)
        self.drone = displayCanvas.create_polygon(drone_rotated_translated_vertices, fill="black")

    def check_crash(self):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        if y<0:
            print("Crashed into ground")
            return False


        return True

    def ensure_circularity(self):
        theta = self.Z[4]
        if theta > math.pi:
            self.Z[4] = self.Z[4] - 2*math.pi
        elif theta < -math.pi:
            self.Z[4] = self.Z[4] + 2*math.pi

    def solve_dynamics(self,dt):
        t = np.array([0, dt])

        self.ensure_circularity()

        self.controller()

        Z = odeint(self.dynamics,self.Z,t)
        Z = Z[-1]

        self.Z = Z

    def dynamics(self,Z,t):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        Izz = (self.m*self.arm_length)/12

        s_theta = math.sin(theta)
        c_theta = math.cos(theta)

        x_dd = -(self.F_r + self.F_l)*s_theta/self.m
        y_dd = ((self.F_r + self.F_l)*c_theta - self.g)/self.m
        theta_dd = (self.F_r - self.F_l)/Izz

        Z_dot = [x_dot,x_dd,y_dot,y_dd,theta_dot,theta_dd]
        return Z_dot

    def controller(self):
        x_ref = self.x_ref
        x_dot_ref = 0
        x_dd_ref = 0
        Kp_x = self.Kp_x
        Kd_x = self.Kd_x
        x = self.Z[0]
        x_dot = self.Z[1]

        y_ref = self.y_ref
        y_dot_ref = 0
        y_dd_ref = 0
        Kp_y = self.Kp_y
        Kd_y = self.Kd_y
        y = self.Z[2]
        y_dot = self.Z[3]

        theta_c_dot = 0
        Kp_theta = self.Kp_theta
        Kd_theta = self.Kd_theta
        theta = self.Z[4]
        theta_dot = self.Z[5]


        theta_c = -(x_dd_ref + Kp_x*(x_ref-x) + Kd_x*(x_dot_ref-x_dot))/self.g
        u1 = self.m*(self.g + y_dd_ref + Kp_y*(y_ref-y) + Kd_y*(y_dot_ref-y_dot))
        u2 = Kp_theta*(theta_c-theta) + Kd_theta*(theta_c_dot-theta_dot)

        self.F_l = (u1-u2)/2
        self.F_r = (u1+u2)/2


    def controller_old(self):
        ref_vec = [4,2,0]
                    # x,y,theta
        P_gain_vec = [0,0,0]
        I_gain_vec = [0,0,0]
        D_gain_vec = [0,0,0]

        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        error_vec = [x-ref_vec[0],y-ref_vec[1],theta-ref_vec[2]]

        # for i in range(3):

        y_error = y_ref - y
        theta_error = theta_ref - theta

        control_law_y = Kp_y*y_error - Kd_y*y_dot
        control_law_theta = Kp_theta*theta_error - Kd_theta*theta_dot

        self.F_l = control_law_y/2 + control_law_theta
        self.F_r = control_law_y/2 - control_law_theta
