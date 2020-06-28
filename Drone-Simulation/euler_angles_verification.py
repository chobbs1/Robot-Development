from sympy import simplify,Symbol,Function,cos,sin,Matrix,Transpose,diff,functions#,multiply_elementwise

t = Symbol("t")
# psi, phi, theta, psi_dot,phi_dot, theta_dot = functions('psi, phi, theta, psi_dot, phi_dot, theta_dot')(t)
phi = Function("phi")(t)
theta = Function("theta")(t)
psi = Function("psi")(t)

phi_dot = diff(phi,t)
theta_dot = diff(theta,t)
psi_dot = diff(psi,t)




Rx = Matrix([
      [1,    0,       0],
      [0,cos(phi),-sin(phi)],
      [0,sin(phi),cos(phi)]
     ])

Ry = Matrix([
      [cos(theta),0,sin(theta)],
      [0,         1,     0],
      [-sin(theta),0,cos(theta)]
     ])

Rz = Matrix([
      [cos(psi),-sin(psi),0],
      [sin(psi),cos(psi),0],
      [0,          0,    1]
     ])

R = Rz*Rx*Ry
R_T = Transpose(R)
R_dot = diff(R,t)

w_b = R_dot*R_T

print(simplify(w_b))
# print(simplify(R_T*R))


# # phi_d = Matrix([[phi_dot],[0],[0]])
# # t_d = Matrix([[0],[theta_dot],[0]])
# # psi_d = Matrix([[0],[0],[psi_dot]])
#
# phi_d = Matrix([[1],[0],[0]])
# t_d = Matrix([[0],[1],[0]])
# psi_d = Matrix([[0],[0],[1]])
#

#
# col1 = Transpose(Ry)
# col2 = t_d
# R31 = Rx*Ry
# print(Transpose(Rx)*Rx)


# R_euler_vels = Matrix([
#     [cos(theta),0,-cos(phi)*sin(theta)],
#     [0,         1, sin(phi)],
#     [sin(theta),0, cos(phi)*cos(theta)]
# ])
#
# w_B = Matrix([
#     [diff(phi,t)],
#     [diff(theta,t)],
#     [diff(psi,t)]
# ])
#
# w_A = R_euler_vels*w_B
# print(w_A)
