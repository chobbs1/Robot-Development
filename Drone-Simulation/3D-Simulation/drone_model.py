from scipy.integrate import odeint
from math import cos,sin,pi
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib.collections import PolyCollection
import sys

class Drone:
    # drone's physical parameters
    L = 0.2
    m = 0.5

    Kf = 1
    Km = 1

    F = [0,0,0,0]
    M = [0,0,0,0]

    w1 = 0
    w2 = 0
    w3 = 0
    w4 = 0
    motor_sat_point = 4000

    # environment variables
    g = 9.81

    # who the fuck knows
    phi0_c = 0
    theta0_c = 0
    psi0_c = 0

    # gains
    Kp = [1,1,3,
          0.1,0.1,0.1]
    Kd = [0,0.3,1,
          0,0,0.001]


    def __init__(self,ICs,dt):
        self.X = ICs
        self.dt = dt
        self.defMomentsInertia()

    def defMomentsInertia(self):
        self.Ixx = 2*self.m*self.L**2/12
        self.Iyy = 2*self.m*self.L**2/12
        self.Izz = 4*self.m*self.L**2/12
        self.I = np.array([[self.Ixx,0,0],[0,self.Iyy,0],[0,0,self.Izz]])

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
        R = np.matmul(Rz,np.matmul(Rx,Ry))


        F_f3 = np.array([[0],[0],[F1+F2+F3+F4]])
        G_f0 = np.array([[0],[0],[-m*g]])
        rOG_dd_f0 = (np.add(np.matmul(R,F_f3),G_f0))/m

        x_dd = rOG_dd_f0[0]
        y_dd = rOG_dd_f0[1]
        z_dd = rOG_dd_f0[2]

        X_dot = [x_dot,y_dot,z_dot,x_dd,y_dd,z_dd]
        return X_dot

    def updateReferences(self,ref):
        self.pos_refs = ref[0:4]
        self.vel_refs = ref[4:8]
        self.acc_refs = ref[8:12]

    def update_controller(self):

        # get all state values
        x = self.X[0]
        y = self.X[1]
        z = self.X[2]
        x_dot = self.X[3]
        y_dot = self.X[4]
        z_dot = self.X[5]
        phi = self.X[6]
        theta = self.X[7]
        psi = self.X[8]
        phi_dot = self.X[9]
        theta_dot = self.X[10]
        psi_dot = self.X[11]

        theta_G = np.array(self.X[6:9])
        w_G = np.array(self.X[9:12])
        w_B = self.rotate2body(theta_G,w_G).tolist()
        p = w_B[0]
        q = w_B[1]
        r = w_B[2]

        # define reference values
        # ref = [0,0,2,0]
        # ref_dot = [0,0,0,0]
        # ref_dd = [0,0,0,0]

        ref = self.pos_refs
        ref_dot = self.vel_refs
        ref_dd = self.acc_refs

        # define gains
        Kp = self.Kp
        Kd = self.Kd

        #calculate commanded linear accelerations
        r1_dd_c = ref_dd[0] + Kd[0]*(ref_dot[0]-x_dot) + Kp[0]*(ref[0]-x)
        r2_dd_c = ref_dd[1] + Kd[1]*(ref_dot[1]-y_dot) + Kp[1]*(ref[1]-y)
        r3_dd_c = ref_dd[2] + Kd[2]*(ref_dot[2]-z_dot) + Kp[2]*(ref[2]-z)
        # print(Kp[0]*(ref[0]-x))

        # calculate commanded ancular positions
        psi_des = ref[3]
        phi_c = (r1_dd_c*sin(psi_des) - r2_dd_c*cos(psi_des))/self.g
        theta_c = (r1_dd_c*cos(psi_des) + r2_dd_c*sin(psi_des))/self.g
        psi_c = psi_des

        # derive commanded ancular positions to get angular rates
        p_c = (phi_c - self.phi0_c)/self.dt
        q_c = (theta_c - self.theta0_c)/self.dt
        r_c = (psi_c - self.psi0_c)/self.dt

        # update old positions to obtain derivatives
        self.phi0_c = phi_c
        self.theta0_c = theta_c
        self.psi0_c = psi_c

        # calculate required hovering forces and moments
        u1 = self.m*(self.g + r3_dd_c)
        u2_x = Kp[3]*(phi_c - phi) + Kd[3]*(p_c - p)
        u2_y = Kp[4]*(theta_c - theta) + Kd[4]*(q_c - q)
        u2_z = Kp[5]*(psi_c - psi) + Kd[5]*(r_c - r)

        # define some new variables for notational simplicity
        a = u2_z/self.Km
        b = u1/self.Kf
        c = u2_x/(self.L*self.Kf)
        d = u2_y/(self.L*self.Kf)

        g1 =  a/4 + b/4 - d/2
        g2 = -a/4 + b/4 + c/2
        g3 =  a/4 + b/4 + d/2
        g4 = -a/4 + b/4 - c/2

        self.w1 = g1**2
        self.w2 = g2**2
        self.w3 = g3**2
        self.w4 = g4**2


    def check_crash(self):
        z = self.X[2]

        if z<0:
            print("Crashed into ground")
            return False
        return True

    def rotate2body(self,theta_G_IC,w_G_IC):
        phi = theta_G_IC[0]
        theta = theta_G_IC[1]

        R = np.array([
                [cos(theta),0,-cos(phi)*sin(theta)],
                [0,1,sin(phi)],
                [sin(theta),0,cos(phi)*cos(theta)]
            ])

        return np.matmul(R,w_G_IC)

    def angular_acc(self,w_B,t):
        F1 = self.Kf*self.w1**2
        F2 = self.Kf*self.w2**2
        F3 = self.Kf*self.w3**2
        F4 = self.Kf*self.w4**2

        M1 = self.Km*self.w1**2
        M2 = self.Km*self.w2**2
        M3 = self.Km*self.w3**2
        M4 = self.Km*self.w4**2

        F = [F1,F2,F3,F4]
        M = [M1,M2,M3,M4]

        inv_I = np.linalg.inv(self.I)
        torques = np.array([self.L*(F[1]-F[3]),
                         self.L*(F[2]-F[0]),
                         M[0]-M[1]+M[2]-M[3]])


        return np.matmul(inv_I,
                    np.subtract(torques,
                        np.cross(w_B,
                            np.matmul(self.I,
                                w_B))))

    def rotate2inertial(self,theta_G_IC,w_B):
        phi = theta_G_IC[0]
        theta = theta_G_IC[1]

        R = np.array([
                [cos(theta),0,-cos(phi)*sin(theta)],
                [0,1,sin(phi)],
                [sin(theta),0,cos(phi)*cos(theta)]
            ])

        return np.matmul(np.linalg.inv(R),w_B)


    def ang_vel_inert(self,X,t):
        w_G = self.X[9:12]
        return w_G

    def solve_dynamics(self):
        t = np.array([0,self.dt])


        # linear dynamics
        rOG_IC = np.array(self.X[0:3])
        rOG_d_IC = np.array(self.X[3:6])

        linear_X = np.concatenate([rOG_IC,rOG_d_IC])

        linear_X = odeint(self.linear_dynamics,linear_X,t)
        linear_X = linear_X[-1]


        # angular dynamics

        theta_G_IC = np.array(self.X[6:9])
        w_G_IC = np.array(self.X[9:12])

        w_B_IC = self.rotate2body(theta_G_IC,w_G_IC)
        w_B = odeint(self.angular_acc,w_B_IC,t)
        w_B = w_B[-1]

        w_G = self.rotate2inertial(theta_G_IC,w_B)
        self.X[9:12] = w_G

        theta_G = odeint(self.ang_vel_inert,theta_G_IC,t)
        theta_G = theta_G[-1]


        # update all
        self.X[:6] = linear_X

        self.X[6:9] = theta_G
        self.X[9:12] = w_G
