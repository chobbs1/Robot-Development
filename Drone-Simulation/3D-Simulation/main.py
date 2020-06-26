from drone_model import Drone

ICs = [0,0,1,0,0,0,0,0,0,0,0,0]

mDrone = Drone(ICs)

t = 0
dt = 0.01

while True:

    mDrone.dynamics(t)


    break
