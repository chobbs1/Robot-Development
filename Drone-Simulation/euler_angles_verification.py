from sympy import symbols,cos,sin,Matrix,Transpose#,multiply_elementwise


psi, phi, theta, psi_dot, phi_dot, theta_dot = symbols('psi, phi, theta, psi_dot, phi_dot, theta_dot')


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

# phi_d = Matrix([[phi_dot],[0],[0]])
# t_d = Matrix([[0],[theta_dot],[0]])
# psi_d = Matrix([[0],[0],[psi_dot]])

phi_d = Matrix([[1],[0],[0]])
t_d = Matrix([[0],[1],[0]])
psi_d = Matrix([[0],[0],[1]])

R = Rz*Rx*Ry

col1 = Transpose(Ry)
col2 = t_d
col3 = Transpose(Rx*Ry)

R_euler_vels = Matrix([
    [cos(theta),0,-cos(phi)*sin(theta)],
    [0,         1, sin(phi)],
    [sin(theta),0, cos(phi)*cos(theta)]
                      ])
