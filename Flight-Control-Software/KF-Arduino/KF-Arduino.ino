#include <Wire.h>
#include <MPU9250.h>

MPU9250 IMU(Wire,0x68);
int status;

long loop_start;
int Ts = 100;
double phi0 = 0, theta0 = 0, tau=50;
double phi_angle,theta_angle;
double phi_z,theta_z,psi_z;
double phi_dot_z,theta_dot_z,psi_dot_z;



void setup() {
 Serial.begin(115200);
 
 
 
 status = IMU.begin();
 initialise_sensors();
 
 
}

void loop() {
  loop_start = millis();
  tau = 0.02;
  
  read_sensors();
  attitude_estimator();

  Serial.print("phi = ");Serial.print(180*phi_angle/3.142);
  Serial.print(" | theta = ");Serial.println(180*theta_angle/3.142);
  

  /*--Extended Kalman Filter--*/
//  compute_control();
//  predict_state();
//  update_state();

  while (millis() - loop_start < Ts) {}
}

/*------------------------------------------------------------------*/


void attitude_estimator() {
  phi_angle = (tau*(phi0+Ts*phi_dot_z) + Ts*phi_z)/(tau+Ts);
  theta_angle = (tau*(theta0+Ts*theta_dot_z) + Ts*theta_z)/(tau+Ts);

  phi0 = phi_angle;
  theta0 = theta_angle;
}


void initialise_sensors() {

//  recalibrate_mag();
  set_mag();
   
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
//  Serial.println("Read Sensors");
  byte AK8963_Address = 0x68;
  IMU.readSensor();
  
  

  phi_z = atan2(IMU.getAccelY_mss(), sqrt(IMU.getAccelX_mss()*IMU.getAccelX_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  theta_z = atan2(IMU.getAccelX_mss(), sqrt(IMU.getAccelY_mss()*IMU.getAccelY_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  psi_z = atan2(IMU.getMagY_uT(),IMU.getMagX_uT());
  
  phi_dot_z = IMU.getGyroX_rads();
  theta_dot_z = IMU.getGyroY_rads();
  psi_dot_z = IMU.getGyroZ_rads();


  
  //  Serial.print("Roll: ");  
//  Serial.print(180*roll/3.142,6);
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
//  Serial.println("Control");
}

void predict_state() {
//  Serial.println("Predict State");
}



void update_state() {
//  Serial.println("Update States");
}



