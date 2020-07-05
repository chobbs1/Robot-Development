import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D

class GUI():

    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim3d([-2.0, 2.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-2.0, 2.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 5.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')



    def init_plot(self):
        self.ax.plot([],[],[],color='blue',linewidth=3)
        self.fig.show()
