import tkinter as tk
import math
import time
from drone_model import Drone
import numpy as np
from scipy.integrate import odeint

# define gui characteristics
WIDTH = 800
HEIGHT_UI = 200
HEIGHT_DISPLAY = 500
SPEED_THRESHOLD = 20

# define physical constants
class Drone:
    arm_length = 20
    arm_width = 2
    m = 1
    g = 9.81
    F_r = 0
    F_l = 0
    int_error = 0


    def __init__(self,ICs):
        self.Z = ICs

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

    def draw_drone(self):
        x = self.Z[0]
        y = self.Z[2]
        theta = self.Z[4]
        arm_length = self.arm_length
        arm_width = self.arm_width

        canvas.delete("all")
        c_theta = math.cos(theta)
        s_theta = math.sin(theta)

        unrotated_vertices = [
            [x - arm_length, y - arm_width],
            [x + arm_length, y - arm_width],
            [x + arm_length, y + arm_width],
            [x - arm_length, y + arm_width],
        ]
        rotated_vertices = []

        for coords in unrotated_vertices:
            x0 = coords[0]
            y0 = coords[1]

            x1 = x0*c_theta - y0*s_theta
            y1 = x0*s_theta + y0*c_theta

            rotated_vertices.append([x1,y1])

        drone = canvas.create_polygon(rotated_vertices, fill="black")

    def check_crash(self):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        if y<0:
            print("Crashed into groud")
            return True
        elif x<0 or x>20:
            print("Off Screen")
            return True
        elif x_dot>SPEED_THRESHOLD or y_dot>SPEED_THRESHOLD:
            print("Unrealistic speeds")
            return True

        return False

    def solve_dynamics(self,dt):

        t = np.arange(0,dt,dt/10)
        self.controller()

        Z = odeint(self.dynamics,self.Z,t)
        self.Z = Z[-1]

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
    #     Z_dot = [x_dot,x_dd,y_dot,y_dd,theta_dot,theta_dd]
    #     return Z_dot

    def dynamics(self,Z,t):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        s_theta = math.sin(theta)
        c_theta = math.cos(theta)

        x_dd = -(self.F_r + self.F_l)*s_theta/self.m
        y_dd = -((self.F_r + self.F_l)*c_theta - self.g)/self.m
        theta_dd = 12*(self.F_r + self.F_l)/(self.m*self.arm_length)

        Z_dot = [x_dot,x_dd,y_dot,y_dd,theta_dot,theta_dd]
        return Z_dot

    def controller(self):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        self.F_l = 1
        self.F_r = 1


gui = tk.Tk()
gui.title("Drone Simulation")
droneFrame = tk.Frame(gui, bg="white", height=HEIGHT_DISPLAY, width=WIDTH)
droneFrame.pack(side=TOP)

uiFrame = tk.Canvas(gui, bg="gray", height=HEIGHT_UI, width=WIDTH)
ui_canvas.pack()

def upKey(event):
    print("Up key pressed")
    mDrone.F_r += 3
    mDrone.F_l += 3

def downKey(event):
    print("Down key pressed")
    mDrone.F_r -= 3
    mDrone.F_l -= 3


gui.bind('<Up>', upKey)
gui.bind('<Down>', downKey)

gui.mainloop()



#-----------------physics------------------------------------
# t = 0
# dt = 0.01
#
# ICs = [250,0,100,0,0,0]
#
# mDrone = Drone(ICs)
#
# i=0
# while i<22002:
#     gui.update_idletasks()
#     gui.update()
#
#
#     mDrone.draw_drone()
#     mDrone.printStateVector()
#     mDrone.solve_dynamics(dt)
#
#
#     t += dt
#
#     time.sleep(dt)
#     # time.sleep(2)
#     if mDrone.check_crash():
#         break
#-----------------physics------------------------------------


# start simulation


#
#
# # mDrone = Drone(250,100,math.pi/10)
#



# def downKey(event):
#     print("Down key pressed")
#
# def leftKey(event):
#     print("Left key pressed")
#
# def rightKey(event):
#     print("Right key pressed")

# frame = Frame(main, width=100, height=100)

# main.bind('<Down>', downKey)
# main.bind('<Left>', leftKey)
# main.bind('<Right>', rightKey)
# frame.pack()
# main.mainloop()

# import tkinter as tk
#
# class App(tk.Tk):
#     def __init__(self):
#         super().__init__()
#         entry = tk.Entry(self)
#         entry.bind("<FocusIn>", self.print_type)
#         entry.bind("<Key>", self.print_key)
#         entry.pack(padx=20, pady=20)
#
#     def print_type(self, event):
#         print(event.type)
#
#     def print_key(self, event):
#         args = event.keysym, event.keycode, event.char
#         print("Symbol: {}, Code: {}, Char: {}".format(*args))
#
# if __name__ == "__main__":
#     app = App()
#     app.mainloop()
