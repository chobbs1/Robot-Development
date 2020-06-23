import tkinter as tk
import math
import time
from drone_model import Drone
import numpy as np
from scipy.integrate import odeint
import sys

# define gui characteristics
SCALE_FACTOR = 100
GUI_WIDTH = 800
WORLD_WIDTH = GUI_WIDTH/SCALE_FACTOR
HEIGHT_UI = 200
HEIGHT_DISPLAY = 500
WORLD_HEIGHT = HEIGHT_DISPLAY/SCALE_FACTOR
SPEED_THRESHOLD = 20
UI_BACKGROUND_COLOUR = "gray"

enVars = [SCALE_FACTOR,HEIGHT_DISPLAY]

# define environment variables


# define physical constants


gui = tk.Tk()
gui.title("Drone Simulation")
gui.geometry("800x700+100+50")

displayCanvas = tk.Canvas(gui, bg="white", height=HEIGHT_DISPLAY, width=GUI_WIDTH)
displayCanvas.pack(side="top")

uiFrame = tk.Frame(gui, bg="gray", height=HEIGHT_UI, width=GUI_WIDTH)
uiFrame.pack(side="bottom")

controlFrame = tk.Frame(uiFrame, bg="gray", height=HEIGHT_UI, width=200)
controlFrame.grid(column=0,row=0)

refFrame = tk.Frame(uiFrame, bg="gray", height=HEIGHT_UI, width=200)
refFrame.grid(column=1,row=0)

gainsFrame = tk.Frame(uiFrame, bg="gray", height=HEIGHT_UI, width=50)
gainsFrame.grid(column=2,row=0)

flightDataFrame = tk.Frame(uiFrame, bg="gray", height=HEIGHT_UI, width=200)
flightDataFrame.grid(column=3,row=0)

ICs = [4,0,2,0,math.pi/100,0]
t = 0
dt = 0.02
#-------------------Buttons for controlling the simulation--------------------
def getGuiInput(mDrone):
    mDrone.y_ref = y_ref_slider.get()
    mDrone.x_ref = x_ref_slider.get()


def startButtonCallback():
    mDrone = Drone(ICs,enVars)
    t = 0
    dt = 0.02

    while mDrone.check_crash():
        getGuiInput(mDrone)

        mDrone.draw_drone(displayCanvas)
        mDrone.solve_dynamics(dt)

        updateFlightData(mDrone,t)

        gui.update()
        gui.update_idletasks()

        # break
        t += dt
        time.sleep(dt)
    mDrone.delete_drone(displayCanvas)


startButton = tk.Button(controlFrame,  bg="white",
        text="Start", fg="black",command = startButtonCallback)
startButton.grid(column=0)

def exitButtonCallback():
    sys.exit()

exitButton = tk.Button(controlFrame,  bg="white",
        text="Quit", fg="black",command = exitButtonCallback)
exitButton.grid(column=0)



logButton = tk.Button(controlFrame,  bg="white", text="Log Data", fg="black")
logButton.grid(column=0)

#----------------------------Type Gain Values-----------------------------
def updateGains():
    mDrone.Kp_x = float(Kp_x.get())
    mDrone.Kd_x = float(Kd_x.get())
    mDrone.Kp_y = float(Kp_y.get())
    mDrone.Kd_y = float(Kd_y.get())
    mDrone.Kp_theta = float(Kp_theta.get())
    mDrone.Kd_theta = float(Kd_theta.get())

updateGainsButton = tk.Button(gainsFrame,  bg="white",
        text="Update Gains", fg="black",command = updateGains)
startButton.grid(column=0,row=0)

tk.Label(gainsFrame,bg="gray", text="X: Kp").grid(column=0,row=1)
Kp_x = tk.StringVar()
Kp_x_entry = tk.Entry(gainsFrame,textvariable=Kp_x,width=20).grid(column=1,row=1)
Kp_x.set(3)

tk.Label(gainsFrame,bg="gray", text="X: Kd").grid(column=0,row=2)
Kd_x = tk.StringVar()
Kd_x_entry = tk.Entry(gainsFrame,textvariable=Kd_x,width=20).grid(column=1,row=2)
Kd_x.set(2)

tk.Label(gainsFrame,bg="gray", text="Y: Kp").grid(column=0,row=3)
Kp_y = tk.StringVar()
Kp_y_entry = tk.Entry(gainsFrame,textvariable=Kp_y,width=20).grid(column=1,row=3)
Kp_y.set(9)

tk.Label(gainsFrame,bg="gray", text="Y: Kd").grid(column=0,row=4)
Kd_y = tk.StringVar()
Kp_x_entry = tk.Entry(gainsFrame,textvariable=Kd_y,width=20).grid(column=1,row=4)
Kd_y.set(3)

tk.Label(gainsFrame,bg="gray", text="Theta: Kp").grid(column=0,row=5)
Kp_theta = tk.StringVar()
Kp_x_entry = tk.Entry(gainsFrame,textvariable=Kp_theta,width=20).grid(column=1,row=5)
Kp_theta.set(0.1)

tk.Label(gainsFrame,bg="gray", text="Theta: Kd").grid(column=0,row=6)
Kd_theta = tk.StringVar()
Kp_x_entry = tk.Entry(gainsFrame,textvariable=Kd_theta,width=20).grid(column=1,row=6)
Kd_theta.set(0.4)






#--------------------------Sliders to set reference---------------------------


y_ref_slider = tk.Scale(refFrame, from_=0, to=5, width=15, length=150,bg=UI_BACKGROUND_COLOUR,
    tickinterval=0.5,orient="horizontal",label="Y reference")
y_ref_slider.set(2)
y_ref_slider.grid(column=2,row=0)


x_ref_slider = tk.Scale(refFrame, from_=0, to=8, width=15, length=150,bg=UI_BACKGROUND_COLOUR,
    tickinterval=0.5,orient="horizontal",label="X Reference")
x_ref_slider.set(4)
x_ref_slider.grid(column=2,row=1)


#-------------------------Flight Status Information-----------------------------
x_pos = tk.StringVar()
x_pos.set("x = {}m".format(ICs[0]))
x_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR,textvariable=x_pos)
x_label.grid(column=3,row=0)

x_vel = tk.StringVar()
x_vel.set("x_dot = {}m/s".format(ICs[1]))
x_dot_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR,textvariable=x_vel)
x_dot_label.grid(column=4,row=0)

y_pos = tk.StringVar()
y_pos.set("y = {}m".format(ICs[2]))
y_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR, textvariable=y_pos)
y_label.grid(column=3,row=1)

y_vel = tk.StringVar()
y_vel.set("y_dot = {}m/s".format(ICs[3]))
y_dot_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR,textvariable=y_vel)
y_dot_label.grid(column=4,row=1)

theta_pos = tk.StringVar()
theta_pos.set("theta = {}rad".format(ICs[4]))
theta_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR,textvariable=theta_pos)
theta_label.grid(column=3,row=2)

theta_vel = tk.StringVar()
theta_vel.set("theta_dot = {}rad/s".format(ICs[5]))
theta_dot_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR,textvariable= theta_vel)
theta_dot_label.grid(column=4,row=2)

time_var = tk.StringVar()
time_var.set("time = %.2fs" % t)
time_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR,textvariable=time_var)
time_label.grid(column=3,row=3)

def updateFlightData(mDrone,t):
    x_pos.set("x = %.2fm" % mDrone.Z[0])
    x_vel.set("x_dot = %.2fm/s" % mDrone.Z[1])
    y_pos.set("y = %.2fm" % mDrone.Z[2])
    y_vel.set("y_dot = %.2fm/s" % mDrone.Z[3])
    theta_pos.set("theta = %.2frad" % mDrone.Z[4])
    theta_vel.set("theta_dot = %.2frad/s" % mDrone.Z[5])
    time_var.set("time = %.2fs" % t)

#-----------------------------------------------------------------------------


def show_values():
    print(P_gain.get())

def upKey(event):
    print("Up key pressed")
    startButtonCallback()
gui.bind('<Up>', upKey)

def downKey(event):
    print("Down key pressed")
gui.bind('<Down>', downKey)

def leftKey(event):
    print("Left key pressed")
gui.bind('<Left>', leftKey)

def rightKey(event):
    print("Right key pressed")
gui.bind('<Right>', rightKey)



#-----------------------------------------------

def draw_compass(displayCanvas):
    x_origin = 10
    y_origin = HEIGHT_DISPLAY-10
    length = 50

    x_arrow = displayCanvas.create_line(x_origin, y_origin, x_origin+length, y_origin,
        arrow=tk.LAST)
    y_arrow = displayCanvas.create_line(x_origin, y_origin, x_origin, y_origin-length,
        arrow=tk.LAST)

    x_origin += 80
    y_origin -= 10
    displayCanvas.create_text(x_origin,y_origin,text="(Xmax = {}m, Ymax = {}m)".format(WORLD_WIDTH,WORLD_HEIGHT))

draw_compass(displayCanvas)

mDrone = Drone(ICs,enVars)
t = 0
dt = 0.02



gui.mainloop()

#-----------------physics------------------------------------

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
