from matplotlib import pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math

class VisualisationTools:
    # plotting parameters
    x_max = 6
    x_min = 4
    y_max = 1
    y_min = -1

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


    def animateRobot(self,x,y,t):
        fig = plt.figure()
        ax = fig.add_subplot()
        print("Starting animation")
        for i in range(0,len(t),self.skip_rate):
            ax.clear()
            robotBody = plt.Circle((x[i],y[i]), 0.2, color='r')
            ax.text(self.x_min+0.1, self.y_min+0.1, "Time = %.2fs" % (t[i]), bbox=dict(facecolor='red', alpha=0.5))

            ax.set(xlabel='X (m)',ylabel='Y (m)',title='Robot in Horizontal Plane',
            xlim=(self.x_min,self.x_max), ylim=(self.y_min,self.y_max))
            # ax.grid()
            ax.add_artist(robotBody)
            plt.pause(1/self.fps)
        plt.close()
