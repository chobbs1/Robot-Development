from drone_model import Drone
from math import pi,cos,sin
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trajectory_planner import Trajectories




def simulate():
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

        arm_array = np.arange(-mDrone.L,mDrone.L+dt,dt)
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

    t = 0
    dt = 0.01
    duration = 10
    time_steps = int(duration/dt)+1

    ICs = [0,1,2,        # rOG
           0,0,0,        # rOG_d
           0,0,0,        # theta
           0,0,0]       # theta_d

    X_T = [0,0,2,        # rOG
           0,0,0,        # rOG_d
           0,0,0,        # theta
           0,0,0]

    mDrone = Drone(ICs,dt)

    traj_plan = Trajectories()
    traj_plan.setCoeff_MinJerkTraj(ICs,X_T,duration)
    fig, ax = init_plot()

    while mDrone.check_crash() and t<duration:

        plot_drone(mDrone.X)

        ref = traj_plan.getReferences(t)
        mDrone.updateReferences(ref)
        mDrone.update_controller()
        mDrone.solve_dynamics()
        t+=dt

        # print("t = %.2f, x = %.2f, y = %.2f, z = %.2f, x_d = %.2f, y_d = %.2f, z_d = %.2f" %
        #     (t,mDrone.X[0],mDrone.X[1],mDrone.X[2],
        #     mDrone.X[3],mDrone.X[4],mDrone.X[5]),end='\r')

        update_plot()


simulate()


def optimise_gains(i):

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

        arm_array = np.arange(-mDrone.L,mDrone.L+dt,dt)
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

    t = 0
    dt = 0.01
    duration = 10
    time_steps = int(duration/dt)+1

    ICs = [0,1,2,        # rOG
           0,0,0,        # rOG_d
           0,0,0,        # theta
           0,0,0]       # theta_d

    X_T = [0,0,2,        # rOG
           0,0,0,        # rOG_d
           0,0,0,        # theta
           0,0,0]

    mDrone = Drone(ICs,dt)
    mDrone.Kp[0] = i

    traj_plan = Trajectories()
    traj_plan.setCoeff_MinJerkTraj(ICs,X_T,duration)
    if i ==3:
        fig, ax = init_plot()
    J = 0
    while mDrone.check_crash() and t<duration:
        if i ==3:
            plot_drone(mDrone.X)

        ref = traj_plan.getReferences(t)
        mDrone.updateReferences(ref)
        mDrone.update_controller()
        mDrone.solve_dynamics()
        t+=dt
        J += (ref[0] - mDrone.X[0])**2
        # print("t = %.2f, x = %.2f, y = %.2f, z = %.2f, x_d = %.2f, y_d = %.2f, z_d = %.2f" %
        #     (t,mDrone.X[0],mDrone.X[1],mDrone.X[2],
        #     mDrone.X[3],mDrone.X[4],mDrone.X[5]),end='\r')

        if i ==3:
            update_plot()
    return J

# optimise_gains(3)
# cost = []
# for i in range(5):
#     cost.append(simulate(i))
# print(cost)
# t_vec = []
# x_ref = []
# x_dot_ref = []
# x_dd_ref = []

# for i in range(time_steps):
#     ref = traj_plan.getReferences(t)
#     t+=dt
#
#     t_vec.append(t)
#     x_ref.append(ref[0])
#     x_dot_ref.append(ref[1])
#     x_dd_ref.append(ref[2])
#
# t_vec = np.array(t_vec)
# x_ref = np.array(x_ref)
# x_dot_ref = np.array(x_dot_ref)
# x_dd_ref = np.array(x_dd_ref)
# #
# # print(t_vec)
# fig, ax = plt.subplots()
# ax.plot(t_vec, x_ref)
# ax.plot(t_vec, x_dot_ref)
# ax.plot(t_vec, x_dd_ref)
# plt.show()
