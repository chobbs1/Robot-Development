from scipy.integrate import odeint
from math import cos,sin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib.collections import PolyCollection

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




    def __init__(self,ICs,dt):
        self.X = ICs
        self.dt = dt
        self.defMomentsInertia()

    def defMomentsInertia(self):
        self.Ixx = 2*self.m*self.L**2/12
        self.Iyy = 2*self.m*self.L**2/12
        self.Izz = 4*self.m*self.L**2/12
        self.I = np.array([[self.Ixx,0,0],[0,self.Iyy,0],[0,0,self.Izz]])

    def body_frame_dynamics(self,w_B,t):
        I_mat = np.matrix(self.I)

        inv_I_mat = np.linalg.inv(I_mat)
        w_B_mat = np.matrix([[w_B[0]],[w_B[1]],[w_B[2]]])


        L = self.L

        M1 = self.Km*self.w1**2
        M2 = self.Km*self.w2**2
        M3 = self.Km*self.w3**2
        M4 = self.Km*self.w4**2

        M_fB_mat = np.matrix([[0],[0],[M1-M2+M3-M4]])





        w_B_mat_arr = np.squeeze(np.asarray(w_B_mat))
        print("w_B_mat_arr:")
        print(w_B_mat_arr)

        I_w_B_arr =  np.squeeze(np.asarray(I_mat*w_B_mat))
        print("I_w_B_arr:")
        print(I_w_B_arr)

        cross_prod_arr = np.cross(w_B_mat_arr,I_w_B_arr)
        print("cross_prod_arr:")
        print(cross_prod_arr)



        w_d_B_mat = inv_I_mat*(M_fB_mat - cross_prod_mat)
        print("w_d_B_mat:")
        print(w_d_B_mat)

        w_d_B_arr = np.squeeze(np.asarray(w_d_B_mat))


        p_dot = w_d_B[0]
        q_dot = w_d_B[1]
        r_dot = w_d_B[2]
        return [p_dot,q_dot,r_dot]

    def angular_dynamics(self,X,t):
        phi = self.X[6]
        theta = self.X[7]
        psi = self.X[8]
        phi_dot = self.X[9]
        theta_dot = self.X[10]
        psi_dot = self.X[11]

        R = np.matrix([
                [cos(theta),0,-cos(phi)*sin(theta)],
                [0,1,sin(phi)],
                [sin(theta),0,cos(phi)*cos(theta)]
            ])

        w_G = np.matrix([[phi_dot],[theta_dot],[psi_dot]])
        w_B = R*w_G

        # print(np.asarray(w_B))
        # .toList()
        w_B = w_B.tolist()
        w_B = [w_B[0][0],w_B[1][0],w_B[2][0]]

        t = np.array([0, self.dt])


        odeint(self.body_frame_dynamics,w_B,t)

        phi_dot = p[-1,:]
        theta_dot = q[-1,:]
        psi_dot = r[-1,:]

        return [phi_dot,theta_dot,psi_dot,phi_dd,theta_dd,psi_dd]


    def linear_dynamics(self,X,t):
        x = self.X[0]
        y = self.X[1]
        z = self.X[2]
        x_dot = self.X[3]
        y_dot = self.X[4]
        z_dot = self.X[5]

        phi = self.X[6]
        theta = self.X[7]
        psi = self.X[8]

        m = self.m
        g = self.g

        F1 = self.Kf*self.w1**2
        F2 = self.Kf*self.w2**2
        F3 = self.Kf*self.w3**2
        F4 = self.Kf*self.w4**2

        Rx = np.matrix([
              [1,    0,       0],
              [0,cos(phi),-sin(phi)],
              [0,sin(phi),cos(phi)]
             ])

        Ry = np.matrix([
              [ cos(theta),0,sin(theta)],
              [ 0,         1,     0],
              [-sin(theta),0,cos(theta)]
             ])

        Rz = np.matrix([
              [cos(psi),-sin(psi),0],
              [sin(psi), cos(psi),0],
              [0,          0,    1]
             ])
        R = Rz*Rx*Ry

        F_f3 = np.matrix([[0],[0],[F1+F2+F3+F4]])
        G_f0 = np.matrix([[0],[0],[-m*g]])
        rOG_dd_f0 = (R*F_f3 + G_f0)/m

        x_dd = rOG_dd_f0[0]
        y_dd = rOG_dd_f0[1]
        z_dd = rOG_dd_f0[2]

        X_dot = [x_dot,y_dot,z_dot,x_dd,y_dd,z_dd]
        return X_dot

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

    def solve_dynamics(self):
        t = np.array([0, self.dt])


        linear_X = self.X[:6]
        # print(linear_X)
        linear_X = odeint(self.linear_dynamics,linear_X,t)
        linear_X = linear_X[-1]

        angular_X = self.X[6:]
        # print(angular_X)
        angular_X = odeint(self.angular_dynamics,angular_X,t)


        # print(angular_X)
        #
        # self.X = [linear_X,angular_X]
        # print(linear_X)


    # def printStateVector(self):
    #     x = self.Z[0]
    #     x_dot = self.Z[1]
    #     y = self.Z[2]
    #     y_dot = self.Z[3]
    #     theta = self.Z[4]
    #     theta_dot = self.Z[5]
    #
    #     print("x = {}".format(x))
    #     print("x_dot = {}".format(x_dot))
    #     print("y = {}".format(y))
    #     print("y_dot = {}".format(y_dot))
    #     print("theta = {}".format(theta))
    #     print("theta_dot = {}".format(theta_dot))
    #     print("F_r = {}".format(self.F_r))
    #     print("F_l = {}".format(self.F_l))
