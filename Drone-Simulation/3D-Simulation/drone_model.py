from scipy.integrate import odeint as Solver
from math import cos,sin
import numpy as np

class Drone:
    # drone's physical parameters
    L = 0.2
    m = 0.5





    Kf = 1
    Km = 0.5

    w1 = 0
    w2 = 0
    w3 = 0
    w4 = 0

    # environment variables
    g = 9.81

    def __init__(self,ICs):
        self.Z = ICs
        self.defMomentsInertia()

    def defMomentsInertia(self):
        self.Ixx = 2*self.m*self.L**2/12
        self.Iyy = 2*self.m*self.L**2/12
        self.Izz = 4*self.m*self.L**2/12
        self.I = np.array([[self.Ixx,0,0],[0,self.Iyy,0],[0,0,self.Izz]])

    def dynamics(self,t):
        x = self.Z[0]
        y = self.Z[1]
        z = self.Z[2]
        phi = self.Z[3]
        theta = self.Z[4]
        psi = self.Z[5]
        x_dot = self.Z[6]
        y_dot = self.Z[7]
        z_dot = self.Z[8]
        phi_dot = self.Z[9]
        theta_dot = self.Z[10]
        psi_dot = self.Z[11]

        I = self.I
        m = self.m
        g = self.g
        L = self.L

        F1 = self.Kf*self.w1
        F2 = self.Kf*self.w2
        F3 = self.Kf*self.w3
        F4 = self.Kf*self.w4

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
        R = Rz*Rx*Ry

        F_f3 = np.array([[0],[0],[F1+F2+F3+F4]])
        G_f0 = np.array([[0],[0],[-m*g]])

        rOG_dd_f0 = R*F_f3 + G_f0

        M1 = Km*self.w1
        M2 = Km*self.w2
        M3 = Km*self.w3
        M4 = Km*self.w4


        M_f3 = np.array([[L*(F2-F4)],[L*(F3-F1)],[M1-M2+M3-M4]])

        R_AB = np.array([[cos(theta),0,-sin(theta)*cos(phi)],
                        [0,1,sin(phi)],
                        [sin(theta),0,cos(phi)*cos(theta)])

        w_A = np.array([[phi_dot],[theta_dot],[psi_dot]])
        w_B = R_AB*w_A

        inv_I = np.linalg.inv(I)

        w_d_Binv_I*(M_f3 - np.cross(w_B, I*w_B))


        Z_dot = [x_dot,x_dd,y_dot,y_dd,theta_dot,theta_dd]
        return Z_dot

    def controller(self):
        x_ref = self.x_ref
        x_dot_ref = 0
        x_dd_ref = 0
        Kp_x = self.Kp_x
        Kd_x = self.Kd_x
        x = self.Z[0]
        x_dot = self.Z[1]

        y_ref = self.y_ref
        y_dot_ref = 0
        y_dd_ref = 0
        Kp_y = self.Kp_y
        Kd_y = self.Kd_y
        y = self.Z[2]
        y_dot = self.Z[3]

        theta_c_dot = 0
        Kp_theta = self.Kp_theta
        Kd_theta = self.Kd_theta
        theta = self.Z[4]
        theta_dot = self.Z[5]


        theta_c = -(x_dd_ref + Kp_x*(x_ref-x) + Kd_x*(x_dot_ref-x_dot))/self.g
        u1 = self.m*(self.g + y_dd_ref + Kp_y*(y_ref-y) + Kd_y*(y_dot_ref-y_dot))
        u2 = Kp_theta*(theta_c-theta) + Kd_theta*(theta_c_dot-theta_dot)

        self.F_l = (u1-u2)/2
        self.F_r = (u1+u2)/2

    def draw_drone(self,displayCanvas):
        x = self.Z[0]
        y = self.Z[2]
        theta = self.Z[4]

        # x_target
        # y_target

        arm_length = self.arm_length
        arm_width = self.arm_width
        motor_height = self.motor_height
        motor_width = self.motor_width

        drone_unrotated_untranslated_vertices = [
            [-arm_length,0],
            [-arm_length,motor_height],
            [-arm_length+motor_width,motor_height],
            [-arm_length+motor_width,arm_width],
            [arm_length-motor_width,arm_width],
            [arm_length-motor_width,motor_height],
            [arm_length,motor_height],
            [arm_length,0],
        ]

        c_theta = math.cos(theta)
        s_theta = math.sin(theta)
        drone_rotated_translated_vertices = []

        for coords in drone_unrotated_untranslated_vertices:
            x0 = coords[0]
            y0 = coords[1]

            x1 = x0*c_theta - y0*s_theta
            y1 = x0*s_theta + y0*c_theta

            x1 = (x1+x)*self.SCALE_FACTOR
            y1 = self.HEIGHT_DISPLAY - (y1+y)*self.SCALE_FACTOR

            drone_rotated_translated_vertices.append([x1,y1])


        try:
            displayCanvas.delete(self.drone)
            # displayCanvas.delete(self.target)
        except AttributeError:
            pass
        r = 5
        # self.target = displayCanvas.create_oval(self.x_ref-r, self.y_ref-r, self.x_ref+r, self.y_ref+r)
        self.drone = displayCanvas.create_polygon(drone_rotated_translated_vertices, fill="black")



    def check_crash(self):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        if y<0:
            print("Crashed into ground")
            return False


        return True

    def ensure_circularity(self):
        theta = self.Z[4]
        if theta > math.pi:
            self.Z[4] = self.Z[4] - 2*math.pi
        elif theta < -math.pi:
            self.Z[4] = self.Z[4] + 2*math.pi

    def solve_dynamics(self,dt):
        t = np.array([0, dt])

        self.ensure_circularity()

        self.controller()

        Z = odeint(self.dynamics,self.Z,t)
        Z = Z[-1]

        self.Z = Z



    def printStateVector(self):
        x = self.Z[0]
        x_dot = self.Z[1]
        y = self.Z[2]
        y_dot = self.Z[3]
        theta = self.Z[4]
        theta_dot = self.Z[5]

        print("x = {}".format(x))
        print("x_dot = {}".format(x_dot))
        print("y = {}".format(y))
        print("y_dot = {}".format(y_dot))
        print("theta = {}".format(theta))
        print("theta_dot = {}".format(theta_dot))
        print("F_r = {}".format(self.F_r))
        print("F_l = {}".format(self.F_l))
