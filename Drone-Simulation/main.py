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

controlFrame = tk.Frame(uiFrame, bg="gray", height=HEIGHT_UI, width=GUI_WIDTH)
controlFrame.pack(side="left")

gainsFrame = tk.Frame(uiFrame, bg="gray", height=HEIGHT_UI, width=GUI_WIDTH)
gainsFrame.pack(side="left")

flightDataFrame = tk.Frame(uiFrame, bg="gray", height=HEIGHT_UI, width=GUI_WIDTH)
flightDataFrame.pack(side="right")


#-------------------Buttons for controlling the simulation--------------------
def startButtonCallback():
    t = 0
    dt = 0.02
    
    ICs = [4,0,3,0,0,0]
    mDrone = Drone(ICs,enVars)

    while mDrone.check_crash():
        mDrone.y_ref = P_gain.get()
        mDrone.draw_drone(displayCanvas)
        mDrone.solve_dynamics(dt)

        gui.update()
        gui.update_idletasks()

        t += dt
        time.sleep(dt)



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

#----------------------------Sliders for Gains-----------------------------
P_gain = tk.Scale(gainsFrame, from_=0, to=10, width=15, length=150,bg=UI_BACKGROUND_COLOUR,
    tickinterval=0.5,orient="horizontal",label="P Gain")
P_gain.set(0)
P_gain.grid(column=2,row=0)


I_gain = tk.Scale(gainsFrame, from_=0, to=10, width=15, length=150,bg=UI_BACKGROUND_COLOUR,
    tickinterval=0.5,orient="horizontal",label="I Gain")
I_gain.set(0)
I_gain.grid(column=2,row=1)


# D_gain = tk.Scale(gainsFrame, from_=0, to=10, width=15, length=150,
#     tickinterval=0.5,orient="horizontal",label="D Gain")
# D_gain.set(0)
# D_gain.grid(column=2,row=2)

#-------------------------Flight Status Information-----------------------------
x = 10
x_dot = 69
y = 11
y_dot = 3
theta = 0
theta_dot = 100
t = 20

x_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR, text="x = {}m".format(x))
x_label.grid(column=3,row=0)

x_dot_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR, justify="left",
        text="x_dot = {}m/s".format(x_dot))
x_dot_label.grid(column=4,row=0)

y_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR, text="y = {}m".format(y))
y_label.grid(column=3,row=1)

y_dot_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR, justify="left",
        text="y_dot = {}m/s".format(y_dot))
y_dot_label.grid(column=4,row=1)

theta_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR, text="theta = {}deg".format(theta))
theta_label.grid(column=3,row=2)

theta_dot_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR, text="theta_dot = {}deg/s".format(theta_dot))
theta_dot_label.grid(column=4,row=2)

time_label = tk.Label(flightDataFrame,bg=UI_BACKGROUND_COLOUR, justify="left",
        text="time = {} s".format(t))
time_label.grid(column=3,row=3)



def show_values():
    print(P_gain.get())

def upKey(event):
    print("Up key pressed")
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
