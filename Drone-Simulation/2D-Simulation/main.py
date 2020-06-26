from drone_model import Drone
from gui import gui
import time


mGui = gui()
enVars = [mGui.SCALE_FACTOR,mGui.DISPLAY_CANVAS_HEIGHT]
ICs = [4,0,2,0,0,0]
initial_gains = [2,8,9,3,0.5,0.9]
mGui.ICs = ICs
mGui.initial_gains = initial_gains
r=5
mDrone = Drone(ICs,enVars)
mDrone.draw_drone(mGui.displayCanvas)

while not mGui.EXIT_PRESSED:
    mGui.gui.update()
    mGui.gui.update_idletasks()

    if mGui.NEW_ICs_AVAIL:
        ICs[0] = mGui.x_IC.get()
        ICs[1] = mGui.x_dot_IC.get()
        ICs[2] = mGui.y_IC.get()
        ICs[3] = mGui.y_dot_IC.get()
        ICs[4] = mGui.theta_IC.get()
        ICs[5] = mGui.theta_dot_IC.get()
        mGui.NEW_ICs_AVAIL = False

    if mGui.RESTART_PRESSED:
        try:
            mGui.displayCanvas.delete(mDrone.drone)
        except AttributeError:
            pass
        print(ICs)
        mDrone.Z = ICs
        mDrone.draw_drone(mGui.displayCanvas)
        mGui.RESTART_PRESSED = False

    if mGui.NEW_GAINS_AVAIL or mGui.t==0:
        mDrone.Kp_x = mGui.Kp_x.get()
        mDrone.Kd_x = mGui.Kd_x.get()
        mDrone.Kp_y = mGui.Kp_y.get()
        mDrone.Kd_y = mGui.Kd_y.get()
        mDrone.Kp_theta = mGui.Kp_theta.get()
        mDrone.Kd_theta = mGui.Kd_theta.get()
        mGui.NEW_GAINS_AVAIL = False


    while mGui.START_PRESSED and not mGui.EXIT_PRESSED:
        mDrone.x_ref = mGui.x_ref_slider.get()
        mDrone.y_ref = mGui.y_ref_slider.get()

        mDrone.draw_drone(mGui.displayCanvas)
        mDrone.solve_dynamics(mGui.dt)


        mGui.updateFlightData(mDrone.Z,mDrone.F_l,mDrone.F_r)
        mGui.incrementTime()
        time.sleep(mGui.dt)

        mGui.gui.update()
        mGui.gui.update_idletasks()
