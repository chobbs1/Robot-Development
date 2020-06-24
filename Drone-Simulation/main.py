from drone_model import Drone
from gui import gui


mGui = gui()
# mDrone = Drone()

while not mGui.EXIT_PRESSED:
    mGui.gui.update()
    mGui.gui.update_idletasks()

    if mGui.NEW_INIT_CON:
        ICs = mGui.getNewICs()
        mGui.NEW_INIT_CON = False


    # while mDrone.check_crash():
    #
    #
    #     mGui.draw_drone(displayCanvas)
    #     mDrone.solve_dynamics(dt)
    #
    #     updateFlightData(mDrone,t)
    #
    #
    #
    #     # break
    #     t += dt
    #     time.sleep(dt)
    # mDrone.delete_drone(displayCanvas)
