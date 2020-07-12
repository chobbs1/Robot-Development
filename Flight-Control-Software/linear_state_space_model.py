from sympy import eye,exp,simplify,Symbol,Function,cos,sin,Matrix,Transpose,diff,functions#,multiply_elementwise


#
# dt = 0.01
#
# A = Matrix([
#       [0,1,0,0,0,0],
#       [0,0,0,0,0,0],
#       [0,0,0,1,0,0],
#       [0,0,0,0,0,0],
#       [0,0,0,0,0,1],
#       [0,0,0,0,0,0],
#      ])
#
# B = Matrix([
#       [0,0,0],
#       [1/Ixx,0,0],
#       [0,0,0],
#       [0,1/Iyy,0],
#       [0,0,0],
#       [0,0,1/Izz],
#      ])
#
# C =  Matrix([1,1,1,1,1,1])
#
#
# Ak = exp(A)*dt
# Bk = A.inv()*(Ak-eye(len(A)))*B
#
# print(Bk)

# t = Symbol("t")
#
# x = Function("x")(t)
# y = Function("y")(t)
# z = Function("z")(t)
#
# x_dot = diff(x,t)
# y_dot = diff(y,t)
# z_dot = diff(z,t)
#
# phi = Function("phi")(t)
# theta = Function("theta")(t)
# psi = Function("psi")(t)
#
# phi_dot = diff(phi,t)
# theta_dot = diff(theta,t)
# psi_dot = diff(psi,t)
#
# F1 = Symbol("F1")
# F2 = Symbol("F2")
# F3 = Symbol("F3")
# F4 = Symbol("F4")
#
# M1 = Symbol("M1")
# M2 = Symbol("M2")
# M3 = Symbol("M3")
# M4 = Symbol("M4")
#
# L = Symbol("L")
#
# # u1_z = Symbol("u1_z")
# # u2_x = Symbol("u2_x")
# # u2_y = Symbol("u2_y")
# # u2_z = Symbol("u2_z")
#
# m = Symbol("m")
# g = Symbol("g")
#
# Rx = Matrix([
#       [1,    0,       0],
#       [0,cos(phi),-sin(phi)],
#       [0,sin(phi),cos(phi)]
#      ])
#
# Ry = Matrix([
#       [cos(theta),0,sin(theta)],
#       [0,         1,     0],
#       [-sin(theta),0,cos(theta)]
#      ])
#
# Rz = Matrix([
#       [cos(psi),-sin(psi),0],
#       [sin(psi),cos(psi),0],
#       [0,          0,    1]
#      ])
#
# R = Rz*Rx*Ry
# print(R)
# U1 = Matrix([
#       [0],
#       [0],
#       [(F1+F2+F3+F4)/m]
#      ])
# g = Matrix([
#       [0],
#       [0],
#       [-g]
#      ])
#
# r_dd = g + R*U1
# # print(r_dd)
#
# # x_dd = (F1+F2+F3+F4)*(sin(phi)*sin(psi)*cos(theta) + sin(theta)*cos(psi))/m
# # y_dd = (F1+F2+F3+F4)*(-sin(phi)*cos(psi)*cos(theta) + sin(psi)*sin(theta))/m
# # z_dd = -g + (F1+F2+F3+F4)*cos(phi)*cos(theta)/m
#
#
#
# R_ang = Matrix([
#       [cos(theta),0,-cos(phi)*sin(theta)],
#       [0         ,1,sin(phi)],
#       [sin(theta),0,cos(phi)*cos(theta)]
#      ])
#
# w_G = Matrix([
#       [phi_dot],
#       [theta_dot],
#       [psi_dot]
#      ])
#
# w_B = R_ang*w_G
#
# # w_B = Matrix([
# # [-sin(theta )*cos(phi )* psi_dot + cos(theta )* phi_dot],
# # [sin(phi )* psi_dot +  theta_dot],
# # [sin(theta )* phi_dot + cos(phi )*cos(theta )* psi_dot]
# # ])
#
# # print(w_B)
#
# Ixx = Symbol("Ixx")
# Iyy = Symbol("Iyy")
# Izz = Symbol("Izz")
#
# I = Matrix([
#       [Ixx,0,  0],
#       [0,  Iyy,0],
#       [0,  0,  Izz]
#      ])
#
# U2 = Matrix([
#       [L*(F2-F4)],
#       [L*(F3-F1)],
#       [M1-M2+M3-M4]
#      ])
#
#
# w_G_dot = I.inv()*(U2 - w_G.cross(I*w_G))
# # print(w_G_dot)
#
# # phi_dd = (Iyy*psi_dot*theta_dot - Izz*psi_dot*theta_dot + L*(F2 - F4))/Ixx
# # theta_dd = (-Ixx*phi_dot*psi_dot + Izz*phi_dot*psi_dot + L*(-F1 + F3))/Iyy
# # psi_dd = (Ixx*phi_dot*theta_dot - Iyy*phi_dot*theta_dot + M1 - M2 + M3 - M4)/Izz
#
#
#
#
#
#
#
# #-------------------------------------------------------------------------------
import control
from control import ss
from control.matlab import c2d,obsv
from numpy.linalg import matrix_rank

Ts = 0.01

Ixx = 1
Iyy = 1
Izz = 1

A = [[0,1,0,0,0,0],
     [0,0,0,0,0,0],
     [0,0,0,1,0,0],
     [0,0,0,0,0,0],
     [0,0,0,0,0,1],
     [0,0,0,0,0,0]]

B = [[0,0,0],
     [1/Ixx,0,0],
     [0,0,0],
     [0,1/Iyy,0],
     [0,0,0],
     [0,0,1/Izz]]

C =  [1,1,1,1,1,1]
D = 0

sys_c = ss(A,B,C,D,Ts)
# sys_d = c2d(sys_c,Ts)

# print(sys_c)

#
# # A = Matrix([[0,1,0,0,0,0,0,0,0,0,0,0],
# #      [0,0,0,0,0,0,0,0,0,0,0,0],
# #      [0,0,0,1,0,0,0,0,0,0,0,0],
# #      [0,0,0,0,0,0,0,0,0,0,0,0],
# #      [0,0,0,0,0,1,0,0,0,0,0,0],
# #      [0,0,0,0,0,0,0,0,0,0,0,0],
# #      [0,0,0,0,0,0,0,1,0,0,0,0],
# #      [0,0,0,0,0,0,0,0,0,0,0,0],
# #      [0,0,0,0,0,0,0,0,0,1,0,0],
# #      [0,0,0,0,0,0,0,0,0,0,0,0],
# #      [0,0,0,0,0,0,0,0,0,0,0,1],
# #      [0,0,0,0,0,0,0,0,0,0,0,0],
# #      ])
#
# A = Matrix([[0,1],
#      [0,0]])
# # # print(A.inv())
# B = [[1],[1],[1],[1],[1],[1],[1],[1],[1],[1],[1],[1]]
# C = [1,0,1,0,1,0,1,1,1,1,1,1]
# #

# # print(sys_d)
# A = [[0,1],[0,0]]
# B = [[0],[1]]
# C = [1,1]
#
# sys_c = ss(A,B,C,D)
# sys_d = c2d(sys_c,Ts)
#
# print(sys_d)
print(matrix_rank(obsv(A,C)))
