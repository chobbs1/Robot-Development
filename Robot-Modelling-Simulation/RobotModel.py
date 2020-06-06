class Robot():
    m = 0.5
    I_G = 1
    x = 0
    x_dot = 0
    y = 0
    y_dot = 0
    theta = 0

    def __init__(self,initial_conditions):
        self.x = initial_conditions[0]
        self.x_dot = initial_conditions[1]
        self.y = initial_conditions[2]
        self.y_dot = initial_conditions[3]
        self.theta = initial_conditions[4]

    def printStatus(self):
        print("Robot Status:")
        print("x = {}".format(self.x))
        print("x_dot = {}".format(self.x_dot))
        print("y = {}".format(self.y))
        print("y_dot = {}".format(self.y_dot))
        print("theta = {}".format(self.theta))

    def updateDynamicModel(x,x_dot,y,y_dot,theta):
        x = 1
        x_dot = 1
        y = 1
        y_dot = 0
        theta = 0

        return None

    def updateControlInputs():
        return None





class MotorModel():
    x = 0
    def __init__(self,initial_conditions):
        self.x = initial_conditions[0]
