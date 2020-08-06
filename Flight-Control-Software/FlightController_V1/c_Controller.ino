/*-------------------Controller Functions------------------------------------------*/
// Gains
double Kp[6] = {0,0,2,1,1,0};
double Kd[6] = {0,0,1,0,0,0};

// Reference Values
double ref[4] = {0,0,0.5,0};
double ref_dot[4] = {0,0,0,0};
double ref_dd[4] = {0,0,0,0};

// controller variables 
double phi0_c=0;
double theta0_c=0;
double psi0_c=0;
double r1_dd_c,r2_dd_c,r3_dd_c;
double phi_c,theta_c,psi_c;
double p_c,q_c,r_c;
double u1,u2_x,u2_y,u2_z;
double w1,w2,w3,w4;
double g1,g2,g3,g4;

const int SAT = 130;


void test_motors() {
   M1.write(Command_RX.MOTOR_SPEED_COMMAND);
   M2.write(Command_RX.MOTOR_SPEED_COMMAND);
   M3.write(Command_RX.MOTOR_SPEED_COMMAND);
   M4.write(Command_RX.MOTOR_SPEED_COMMAND);
}

void zero_motors() {
   M1.write(0);
   M2.write(0);
   M3.write(0);
   M4.write(0);
}


void update_rotation_mat() {
  double c_phi,s_phi,c_theta,s_theta,c_psi,s_psi;
  c_phi = cos(phi);
  s_phi = sin(phi);
  c_theta = cos(theta);
  s_theta = sin(theta);
  c_psi = cos(psi);
  s_psi = sin(psi);

  R[0][0] = -s_phi*s_psi*s_theta + c_psi*c_theta;
  R[0][1] = -s_psi*c_phi;
  R[0][2] = s_phi*s_psi*c_theta + s_theta*c_psi;
  R[1][0] = s_phi*s_theta*c_psi + s_psi*c_theta;
  R[1][1] = c_phi*c_psi;
  R[1][2] = -s_phi*c_psi*c_theta + s_psi*s_theta;
  R[2][0] = -s_theta*c_phi;
  R[2][1] = s_phi;
  R[2][2] = c_phi*c_theta;
}

void compute_control() {
  r1_dd_c = ref_dd[0] + Kd[0]*(ref_dot[0]-x_dot) + Kp[0]*(ref[0]-x);
  r2_dd_c = ref_dd[1] + Kd[1]*(ref_dot[1]-y_dot) + Kp[1]*(ref[1]-y);
  r3_dd_c = ref_dd[2] + Kd[2]*(ref_dot[2]-z_dot) + Kp[2]*(ref[2]-z);
  

  phi_c = (r1_dd_c*sin(ref[3]) - r2_dd_c*cos(ref[3]))/G;
  theta_c = (r1_dd_c*cos(ref[3]) + r2_dd_c*sin(ref[3]))/G;
  psi_c = ref[3];

  p_c = (phi_c - phi0_c)/Ts;
  q_c = (theta_c - theta0_c)/Ts;
  r_c = (psi_c - psi0_c)/Ts;


  phi0_c = phi_c;
  theta0_c = theta_c;
  psi0_c = psi_c;


  u1 = mass*(G + r3_dd_c);
  u2_x = Kp[3]*(phi_c - phi) + Kd[3]*(p_c - phi);
  u2_y = Kp[4]*(theta_c - theta) + Kd[4]*(q_c - theta);
  u2_z = Kp[5]*(psi_c - psi) + Kd[5]*(r_c - psi);

//  Serial.print(u1,2);Serial.print(",");
//  Serial.print(u2_x,2);Serial.print(",");
//  Serial.print(u2_y,2);Serial.print(",");
//  Serial.println(u2_z,2);

  g1 = u2_z/(4*Km) + u1/(4*Kf) - u2_y/(2*Kf*L);
  g2 = -u2_z/(4*Km) + u1/(4*Kf) + u2_x/(2*Kf*L);
  g3 = u2_z/(4*Km) + u1/(4*Kf) + u2_y/(2*Kf*L);
  g4 = -u2_z/(4*Km) + u1/(4*Kf) - u2_x/(2*Kf*L);

  if(g1>=0) {
    w1 = sqrt(g1); 
  } else if(g1>SAT) {
    w1 = SAT;
  }

  if(g2>=0) {
    w2 = sqrt(g2); 
  }else if(g2*g2>SAT) {
    w2 = SAT;
  }

  if(g3>=0) {
    w3 = sqrt(g3); 
  } else if(g3*g3>SAT) {
    w3 = SAT;
  }

  if(g4>=0) {
    w4 = sqrt(g4); 
  } else if(g4*g4>SAT) {
    w4 = SAT;
  }

  Serial.print(w1,2);Serial.print(",");
  Serial.print(w2,2);Serial.print(",");
  Serial.print(w3,2);Serial.print(",");
  Serial.println(w4,2);


  M1.write(w1);
  M2.write(w2);
  M3.write(w3);
  M4.write(w4);
}
