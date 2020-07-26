import numpy as np

print()

Rt = []
Qt = []

def poll_sensors():
    x = 0 + np.random.normal(0,1)
    y = 0 + np.random.normal(0,1)
    z = 0 + np.random.normal(0,1)
    x_dot = 0 + np.random.normal(0,1)
    y_dot = 0 + np.random.normal(0,1)
    z_dot = 0 + np.random.normal(0,1)
    psi = 0 + np.random.normal(0,1)
    psi_dot = 0 + np.random.normal(0,1)

    Zt = [x,y,z,x_dot,y_dot,z_dot,psi,psi_dot]
    return Zt

[-s_phi*s_psi*s_theta + c_psi*c_theta, -s_psi*c_phi, s_phi*s_psi*c_theta + s_theta*c_psi],
[s_phi*s_theta*c_psi + s_psi*c_theta, c_phi*c_psi, -s_phi*c_psi*c_theta + s_psi*s_theta],
[-s_theta*c_phi, s_phi, c_phi*c_theta]

def compute_control():
    Ut = [0,0,0,0]
    return Ut

def g(Ut,Xt0):

    return X1

def jac(Ut,Xt0,dt):

    return At

def predict_state(Xt0,Pt,Ut,dt):
    Xt1 = g(Ut,Xt0)
    At = jac(Ut,Xt0,dt)
    Pt1 = At*Pt0*transpose(At) + Qt
    return Xt1, Pt1


def update_state(Xt0,Pt0,Zt):
    Ht = eye(7)
    Kt = Pt0*transpose(Ht)*inv(Ht*Pt0*transpose(Ht)+Rt)
    Xt1 = Xt0 + Kt*(Zt - Xt0)
    Pt1 = (I - Kt*Ht)*Pt0
    return Xt1,Pt1


def ExtendedKalmanFilter():
    Ut = compute_control(Xt,Pt)
    Xt, Pt = predict_state(Xt,Pt,Ut,dt)
    Zt = poll_sensors()
    Xt,Pt = update_state(Xt,Pt,Zt)
