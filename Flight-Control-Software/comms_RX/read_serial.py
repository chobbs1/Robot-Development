import serial
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from math import pi,cos,sin
import datetime

L = 0.223
dt = 0.01

def init_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d([-1.0, 1.0])
    ax.set_xlabel('X')
    ax.set_ylim3d([-1.0, 1.0])
    ax.set_ylabel('Y')
    ax.set_zlim3d([0, 5.0])
    ax.set_zlabel('Z')
    fig.show()
    return fig, ax

def update_plot():
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(dt)
    ax.clear()
    ax.set_xlim3d([-1.0, 1.0])
    ax.set_xlabel('X')
    ax.set_ylim3d([-1.0, 1.0])
    ax.set_ylabel('Y')
    ax.set_zlim3d([0, 5.0])
    ax.set_zlabel('Z')

def getRotation(X):
    phi = X[6]
    theta = X[7]
    psi = X[8]

    Rx = np.array([
          [1,    0,       0],
          [0,cos(phi),-sin(phi)],
          [0,sin(phi),cos(phi)]
         ])

    Ry = np.array([
          [ cos(theta),0,sin(theta)],
          [ 0,         1,     0],
          [-sin(theta),0,cos(theta)]
         ])

    Rz = np.array([
          [cos(psi),-sin(psi),0],
          [sin(psi), cos(psi),0],
          [0,          0,    1]
         ])
    return np.dot(Rz,np.dot(Rx,Ry))

def plot_drone(X):
    x = X[0]
    y = X[1]
    z = X[2]

    arm_array = np.arange(-L,L+dt,dt)
    zeros = np.zeros(len(arm_array))

    R = getRotation(X)

    arm1 = []
    arm2 = []

    rot_arm1_x = []
    rot_arm1_y = []
    rot_arm1_z = []

    rot_arm2_x = []
    rot_arm2_y = []
    rot_arm2_z = []

    for i in range(len(arm_array)):
        arm1 = np.array([arm_array[i],0,0])
        arm2 = np.array([0,arm_array[i],0])


        rot_arm1 = np.matmul(R,arm1)
        rot_arm2 = np.matmul(R,arm2)

        rot_arm1_x.append(rot_arm1[0]+x)
        rot_arm1_y.append(rot_arm1[1]+y)
        rot_arm1_z.append(rot_arm1[2]+z)

        rot_arm2_x.append(rot_arm2[0]+x)
        rot_arm2_y.append(rot_arm2[1]+y)
        rot_arm2_z.append(rot_arm2[2]+z)


    arm1 = ax.plot(rot_arm1_x,rot_arm1_y,rot_arm1_z,
            color='blue',linewidth=3,antialiased=False)
    arm2 = ax.plot(rot_arm2_x,rot_arm2_y,rot_arm2_z,
            color='red',linewidth=3,antialiased=False)

def readSerial():
    b = ser.readline()
    string_n = b.decode()
    string = string_n.rstrip()
    X = string.split(",")
    Z = [float(X[1]),float(X[2]),float(X[3]),
        0,0,0,float(X[4]),float(X[5]),float(X[6]),0,0,0]
    return Z,float(X[0])



ser = serial.Serial('COM3', 115200)
fig, ax = init_plot()

print("Intialising Timers")
CONTINUE_FLAG = True
while CONTINUE_FLAG:
    try:
        B,drone_start_time = readSerial()
        CONTINUE_FLAG = False
    except UnicodeDecodeError:
        pass

start_loop = time.time()


print(drone_start_time)


THRESHOLD = 10
count = 0

while True:
    ser.flushInput()
    try:

        Z,drone_time = readSerial()
        loop_time = 1000*(time.time() - start_loop)
        drone_time = drone_time - drone_start_time

        # print("PyLoop: %.2f | DroneLoop: %.2f" % (loop_time,drone_time))
        print(drone_time-loop_time)
        # print(Z)
        count += 1

        # time.sleep(dt)
    except UnicodeDecodeError:
        pass
    except IndexError:
        pass

    if count==THRESHOLD:
        count = 0
        plot_drone(Z)
        update_plot()
    # while datetime.datetime.now().second - a.microsecond < 20000:
    #     pass


    c = datetime.datetime.now()
    # print("loop time:",c-a)
    # time.sleep(dt)            # wait (sleep) 0.1 seconds

ser.close()

for line in data:
    print(line)

# import matplotlib.pyplot as plt
#
#
# plt.plot(data)
# plt.xlabel('Time (seconds)')
# plt.ylabel('Potentiometer Reading')
# plt.title('Potentiometer Reading vs. Time')
# plt.show()
