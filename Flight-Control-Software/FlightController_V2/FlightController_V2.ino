#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <MatrixMath.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define ACTUATORS 4
#define STATES 7
#define ROT 3
#define VEC 1

MPU9250 IMU(Wire,0x68);
Adafruit_BMP280 bmp;
int status;
byte AK8963_Address = 0x68;

// misc parameters
long loop_start;
double Ts = 0.01;
double phi0 = 0, theta0 = 0;

// tunable parameters
double tau = 2;

// current state estimates
double x,y,z;
double x_dot,y_dot,z_dot;
double phi,theta,psi;
double phi_dot,theta_dot,psi_dot;

// sensor measurements
double x_meas,y_meas,z_meas;
double x_dot_meas = 0,y_dot_meas = 0,z_dot_meas = 0;
double phi_z,theta_z,psi_z;
double phi_dot_z,theta_dot_z,psi_dot_z;

// calibration values
double z_bias;

// define matrices
mtx_type R[ROT][ROT];
mtx_type R_prime[ROT][ROT] = {
    {0,0,0},
    {0,0,0},
    {0,0,0}
};

mtx_type mu[STATES][VEC];

mtx_type U[ACTUATORS][VEC];

mtx_type SIG[STATES][STATES] = {
  {1,0,0,0,0,0,0},
  {0,1,0,0,0,0,0},
  {0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0},
  {0,0,0,0,1,0,0},
  {0,0,0,0,0,1,0},
  {0,0,0,0,0,0,1}};






  



void setup() {
 Serial.begin(115200);
 Serial.println("Begin");
 initialise_sensors();
 calibrate_sensors();
 Serial.println("Sensors calibrated");
}


void loop() {
  loop_start = millis();
  Serial.println(loop_start);
  /*--Extended Kalman Filter--*/
//  compute_control();
  attitude_estimator();

  predict_state();
  read_sensors();
//  update_state();

  while (millis() - loop_start < 100) {}
}

/*------------------------------------------------------------------*/
void update_rotation_mat() {
//  Serial.println("update rot matrix");
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

  R_prime[0][0] = -s_phi*s_theta*c_psi - s_psi*c_theta;
  R_prime[0][1] = -c_phi*c_psi;
  R_prime[0][2] = s_phi*c_psi*c_theta - s_psi*s_theta;
  R_prime[1][0] = -s_phi*s_psi*s_theta + c_psi*c_theta;
  R_prime[1][1] = -s_psi*c_phi;
  R_prime[1][2] = s_phi*s_psi*c_theta + s_theta*c_psi;
}

void predict_state() {
//  Serial.println("predict_state");
  update_rotation_mat();

  mtx_type g_prime[STATES][STATES] = {
  {1,0,0,Ts,0,0,0},
  {0,1,0,0,Ts,0,0},
  {0,0,1,0,0,Ts,0},
  {0,0,0,1,0,0,0},
  {0,0,0,0,1,0,0},
  {0,0,0,0,0,1,0},
  {0,0,0,0,0,0,1},
};
  
  // update transition matrix
  mu[0][0] = x + x_dot*Ts;
  mu[1][0] = y + y_dot*Ts;
  mu[2][0] = z + z_dot*Ts;
  mu[3][0] = x_dot + Ts*(R[0][0]*U[0][0] + R[0][1]*U[1][0] + R[0][2]*U[2][0]);
  mu[4][0] = y_dot + Ts*(R[1][0]*U[0][0] + R[1][1]*U[1][0] + R[1][2]*U[2][0]);
  mu[5][0] = z_dot + Ts*(R[2][0]*U[0][0] + R[2][1]*U[1][0] + R[2][2]*U[2][0]);
  mu[6][0] = psi + Ts*U[3][0];
  

  // update jacobian matrix
  g_prime[3][6] = Ts*(R_prime[0][0]*U[0][0] + R_prime[0][1]*U[1][0] + R_prime[0][2]*U[2][0]);
  g_prime[4][6] = Ts*(R_prime[1][0]*U[0][0] + R_prime[1][1]*U[1][0] + R_prime[1][2]*U[2][0]);
  g_prime[5][6] = Ts*(R_prime[2][0]*U[0][0] + R_prime[2][1]*U[1][0] + R_prime[2][2]*U[2][0]);
  
  // update covariance matrix: ¯Σt = (Gt * Σt−1 * GT) + Qt
  mtx_type Q[STATES][STATES] = {
  {1,0,0,0,0,0,0},
  {0,1,0,0,0,0,0},
  {0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0},
  {0,0,0,0,1,0,0},
  {0,0,0,0,0,1,0},
  {0,0,0,0,0,0,1}};
  mtx_type temp1[STATES][STATES];

  Matrix.Multiply((mtx_type*)g_prime, (mtx_type*)SIG, STATES, STATES, STATES, (mtx_type*)temp1);
  Matrix.Transpose((mtx_type*) g_prime, STATES, STATES, (mtx_type*) g_prime);
  Matrix.Multiply((mtx_type*)temp1, (mtx_type*)g_prime, STATES, STATES, STATES, (mtx_type*)temp1);
  Matrix.Add((mtx_type*) temp1,(mtx_type*) Q, STATES, STATES, (mtx_type*) SIG);
}

void update_state() {
  // compute kalman gain
  mtx_type Kt[STATES][STATES];
  double Rt[STATES] = {1,1,1,1,1,1,1};
  mtx_type SIG_plus_Rt[STATES][STATES];

  for(int i=0;i<STATES;i++) {
    for(int j=0;j<STATES;j++) {
      if(i==j) {
        SIG_plus_Rt[i][j] = SIG[i][j] + Rt[i];
      } 
      else {
        SIG_plus_Rt[i][j] = SIG[i][j];
      }
    }
  }

  Matrix.Invert((mtx_type*)SIG_plus_Rt, STATES);
  Matrix.Multiply((mtx_type*)SIG, (mtx_type*)SIG_plus_Rt, STATES, STATES, STATES, (mtx_type*)Kt);



  // update state
  mtx_type state_diff[STATES][VEC] = {{x_meas-x},{y_meas-y},{z_meas-z},{x_dot_meas-x_dot},
                                {y_dot_meas-y_dot},{z_dot_meas-z_dot},{psi_z-psi}};

  mtx_type temp1[STATES][VEC];
  Matrix.Multiply((mtx_type*)Kt, (mtx_type*)state_diff, STATES, STATES, VEC, (mtx_type*)temp1);
  Matrix.Add((mtx_type*) temp1, (mtx_type*) mu, STATES, STATES, (mtx_type*) mu);


  // update covariance
  for(int i=0;i<STATES;i++) {
    for(int j=0;j<STATES;j++) {
      if(i==j) {
        Kt[i][j] = 1 - Kt[i][j];
      } 
      else {
        Kt[i][j] = Kt[i][j];
      }
    }
  }
  
  Matrix.Multiply((mtx_type*)Kt, (mtx_type*)SIG, STATES, STATES, STATES, (mtx_type*)SIG);


  // update easy to read globals
  x = mu[0][0];
  x_dot = mu[1][0];
  y = mu[2][0];
  y_dot = mu[3][0];
  z = mu[4][0];
  z_dot = mu[5][0];
  psi = mu[6][0];
}

void add_SIG_and_Rt() {
  
}




//void print_matrix(double **matrix,int row,int col) {
//  Serial.println(&&matrix);
//  Serial.println("\nOutput Matrix\n[");
//  for(int i=0;i<row;i++) {
//    Serial.print("[");
//    for(int j=0;j<col;j++) {
//      Serial.print(matrix[i][j]);
//      Serial.print(" ");
//    }
//    Serial.println("]");
//  }
//  Serial.println();
//  delay(1000);
//}

void attitude_estimator() {
  phi = (tau*(phi0+Ts*phi_dot_z) + Ts*phi_z)/(tau+Ts);
  theta = (tau*(theta0+Ts*theta_dot_z) + Ts*theta_z)/(tau+Ts);

  phi0 = phi;
  theta0 = theta;
}


void initialise_sensors() {
  status = IMU.begin();
//  recalibrate_mag();
  set_mag();

  bmp.begin();
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  for (int i=0;i<5;i++) {
    bmp.readAltitude(1026.7);
  }
  z_bias = bmp.readAltitude(1026.7);
  
   
}

void calibrate_sensors() {
  
}

void recalibrate_mag() {
  Serial.println("Start fig 8");
  status = IMU.calibrateMag();
  Serial.println("End fig 8");
  
  Serial.println(IMU.getMagBiasX_uT());
  Serial.println(IMU.getMagScaleFactorX());
  Serial.println(IMU.getMagBiasY_uT());
  Serial.println(IMU.getMagScaleFactorY());
  Serial.println(IMU.getMagBiasZ_uT());
  Serial.println(IMU.getMagScaleFactorZ());
}

void set_mag() {
  IMU.setMagCalX(29.97,1.10);
  IMU.setMagCalY(37.81,0.89);
  IMU.setMagCalZ(-31.72,1.04);  
}


void read_sensors() {  
  IMU.readSensor();
  
  phi_z = atan2(IMU.getAccelY_mss(), sqrt(IMU.getAccelX_mss()*IMU.getAccelX_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  theta_z = atan2(IMU.getAccelX_mss(), sqrt(IMU.getAccelY_mss()*IMU.getAccelY_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  psi_z = atan2(IMU.getMagY_uT(),IMU.getMagX_uT());
  
  phi_dot_z = IMU.getGyroX_rads();
  theta_dot_z = IMU.getGyroY_rads();
  psi_dot_z = IMU.getGyroZ_rads();

  double x0 = x_meas;
  double y0 = y_meas;
  double z0 = z_meas;
  
  // add GPS in here when available
  x_meas = 0;
  y_meas = 0;
  z_meas = bmp.readAltitude(1026.7) - z_bias;

  x_dot_meas = (x_meas-x0)/Ts;
  y_dot_meas = (y_meas-y0)/Ts;
  z_dot_meas = (z_meas-z0)/Ts;

  
//  Serial.print("Z: ");  
//  Serial.println(z_meas);
//  Serial.print(", Pitch: ");  
//  Serial.print(180*pitch/3.142,6);


//  double mag_x = IMU.getMagX_uT()*cos(pitch) + IMU.getMagY_uT()*sin(roll)*sin(pitch) + IMU.getMagZ_uT()*cos(roll)*sin(pitch);
//  double mag_y = IMU.getMagY_uT()*cos(roll) - IMU.getMagZ_uT() * sin(roll);
  

//  double yaw = atan(IMU.getMagZ_uT()/sqrt(IMU.getMagX_uT()*IMU.getMagX_uT() + IMU.getMagZ_uT()*IMU.getMagZ_uT()));
  
//  Serial.print(", Yaw: ");
//  Serial.print("Yaw: ");  
//  Serial.println(180*yaw/3.142,6);
  
//  Serial.print("AccelX: ");
//  Serial.print(IMU.getAccelX_mss(),6);
//  Serial.print("  ");
//  Serial.print("AccelY: ");  
//  Serial.print(IMU.getAccelY_mss(),6);
//  Serial.print("  ");
//  Serial.print("AccelZ: ");  
//  Serial.println(IMU.getAccelZ_mss(),6);

//  Serial.print("GyroX: ");
//  Serial.print(IMU.getGyroX_rads(),6);
//  Serial.print("  ");
//  Serial.print("GyroY: ");  
//  Serial.print(IMU.getGyroY_rads(),6);
//  Serial.print("  ");
//  Serial.print("GyroZ: ");  
//  Serial.println(IMU.getGyroZ_rads(),6);

//  Serial.print("MagX: ");  
//  Serial.print(IMU.getMagX_uT(),6);
//  Serial.print("  ");  
//  Serial.print("MagY: ");
//  Serial.print(IMU.getMagY_uT(),6);
//  Serial.print("  ");
//  Serial.print("MagZ: ");  
//  Serial.println(IMU.getMagZ_uT(),6);

//  Serial.print("B: ");  
//  Serial.println(sqrt(pow(IMU.getMagX_uT(),2)+pow(IMU.getMagY_uT(),2)+pow(IMU.getMagZ_uT(),2)),6);

//    Serial.print("Temperature in C: ");
//    Serial.println(IMU.getTemperature_C(),6);
}






void compute_control() {
  U[0][0] = 1;
  U[1][0] = 1;
  U[2][0] = 1;
  U[3][0] = 1;
}





