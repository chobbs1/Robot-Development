import numpy as np

class Trajectories:

    def getJerkMatrix(self,a,b,T):

        A = np.array([a,b,0,0,0,0])
        B = np.array([
            [0,0,0,0,0,1],
            [T**5,T**4,T**3,T**2,T,1],
            [0,0,0,0,1,0],
            [5*T**4,4*T**3,3*T**2,2*T,1,0],
            [0,0,0,2,0,0],
            [20*T**3,12*T**2,6*T,2,0,0]
        ])

        inv_B = np.linalg.inv(B)
        return np.matmul(inv_B,A).tolist()

    def setCoeff_MinJerkTraj(self,X_0,X_T,T):
        self.X_0 = X_0
        self.X_T = X_T

        x0 = X_0[0]
        xT = X_T[0]
        self.x_coeffs = self.getJerkMatrix(x0,xT,T)

        y0 = X_0[1]
        yT = X_T[1]
        self.y_coeffs = self.getJerkMatrix(y0,yT,T)

        z0 = X_0[2]
        zT = X_T[2]
        self.z_coeffs = self.getJerkMatrix(z0,zT,T)

        psi0 = X_0[8]
        psiT = X_T[8]
        self.psi_coeffs = self.getJerkMatrix(psi0,psiT,T)

    def getPos(self,coeffs,t):
        [a5,a4,a3,a2,a1,a0] = coeffs
        pos = a5*t**5 + a4*t**4 + a3*t**3 + a2*t**2 + a1*t + a0
        return pos

    def getVel(self,coeffs,t):
        [a5,a4,a3,a2,a1,a0] = coeffs
        vel = 5*a5*t**4 + 4*a4*t**3 + 3*a3*t**2 + 2*a2*t + a1
        return vel

    def getAcc(self,coeffs,t):
        [a5,a4,a3,a2,a1,a0] = coeffs
        acc = 20*a5*t**3 + 12*a4*t**2 + 6*a3*t + 2*a2
        return acc

    def getReferences(self,t):
        x = self.getPos(self.x_coeffs,t)
        x_dot = self.getVel(self.x_coeffs,t)
        x_dd = self.getAcc(self.x_coeffs,t)

        y = self.getPos(self.y_coeffs,t)
        y_dot = self.getVel(self.y_coeffs,t)
        y_dd = self.getAcc(self.y_coeffs,t)

        z = self.getPos(self.z_coeffs,t)
        z_dot = self.getVel(self.z_coeffs,t)
        z_dd = self.getAcc(self.z_coeffs,t)

        psi = self.getPos(self.psi_coeffs,t)
        psi_dot = self.getVel(self.psi_coeffs,t)
        psi_dd = self.getAcc(self.psi_coeffs,t)

        ref = [x,x_dot,x_dd,y,y_dot,y_dd,z,z_dot,z_dd,psi,psi_dot,psi_dd]
        return ref
