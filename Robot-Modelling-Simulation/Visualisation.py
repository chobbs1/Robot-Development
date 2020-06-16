from matplotlib import pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math

class VisualisationTools:
    # plotting parameters
    x_max = -6
    x_min = 6
    y_max = 6
    y_min = -6

    time = 6
    fps = 60
    dt = 0.01
    total_frames = time/dt
    skip_rate = int(total_frames/fps)

    def __init__(self):
        return None

    def plot_vs_time(self,t,x,xlabel='Time (s)',ylabel='',title='Test Plotting'):
        fig, ax = plt.subplots()

        ax.plot(t, x)
        ax.set(xlabel=xlabel,ylabel=ylabel,title=title)
        ax.grid()
        plt.show()
        plt.savefig("Visualisation/Plots/"+title+".png")

    def plot_state_vs_time(self,t,Z):
        x = Z[:,0]
        y = Z[:,1]
        theta = Z[:,2]
        x_dot = Z[:,3]
        y_dot = Z[:,4]
        theta_dot = Z[:,5]

        fig, ax = plt.subplots(2,3,figsize=(12, 7))

        ax[0, 0].plot(t, x)
        ax[0, 0].set(ylabel="X (m)",title="X vs Time")
        ax[0, 0].grid()

        ax[1, 0].plot(t, x_dot)
        ax[1, 0].set(xlabel="Time (s)",ylabel="X_dot (m/s)",title="X_dot vs t")
        ax[1, 0].grid()

        ax[0, 1].plot(t, y)
        ax[0, 1].set(ylabel="Y (m)",title="Y vs Time")
        ax[0, 1].grid()

        ax[1, 1].plot(t, y_dot)
        ax[1, 1].set(xlabel="Time (s)",ylabel="Y_dot (m/s)",title="Y_dot vs t")
        ax[1, 1].grid()

        ax[0, 2].plot(t, theta)
        ax[0, 2].set(ylabel="Theta (rad)",title="Theta vs Time")
        ax[0, 2].grid()

        ax[1, 2].plot(t, theta_dot)
        ax[1, 2].set(xlabel="Time (s)",ylabel="Theta_dot (rad/s)",title="Theta_dot vs t")
        ax[1, 2].grid()


        plt.show()


    def animateRobot(self,x,y,theta,t):
        fig = plt.figure()
        ax = fig.add_subplot()
        print("Starting animation")
        for i in range(0,len(t),self.skip_rate):
            ax.clear()
            robotBody = plt.Circle((x[i],y[i]), 0.2, color='r')
            ax.text(self.x_min-0.1, self.y_min-0.1, "Time = %.2fs" % (t[i]), bbox=dict(facecolor='red', alpha=0.5))

            ax.set(xlabel='X (m)',ylabel='Y (m)',title='Robot in Horizontal Plane',
            xlim=(self.x_min,self.x_max), ylim=(self.y_min,self.y_max))
            # ax.grid()
            ax.add_artist(robotBody)
            plt.pause(1/self.fps)
        # plt.close()
