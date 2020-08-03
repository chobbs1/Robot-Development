/*----------------------Sensor Fusion Functions-----------------------------------------*/
// tunable parameters
double tau = 0.02;
double tau_z = 0.01;
double air_press = 1029.1;

// sensor measurement variables

double x_meas0,y_meas0,z_meas0;
double x_dot_meas = 0,y_dot_meas = 0,z_dot_meas = 0;


void read_sensors() {  
  IMU.readSensor();

  // ***Make sure that these align with the actual axes of the drone
  phi_z = -atan2(IMU.getAccelX_mss(), sqrt(IMU.getAccelY_mss()*IMU.getAccelY_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  theta_z = atan2(IMU.getAccelY_mss(), sqrt(IMU.getAccelX_mss()*IMU.getAccelX_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  psi_z = atan2(IMU.getMagY_uT(),IMU.getMagX_uT());

  phi_z -= phi_bias;
  theta_z -= theta_bias;
  psi_z -= psi_bias; 
  


  
  phi_dot_z = -IMU.getGyroX_rads();
  theta_dot_z = IMU.getGyroY_rads();
  psi_dot_z = IMU.getGyroZ_rads();

  phi_dot_z -= phi_d_bias;
  theta_dot_z -= theta_d_bias;
  psi_dot_z -= psi_d_bias;



//  read_GPS();  
//  x_meas = getX(lat1,lon1) - x_offset;
//  y_meas = getY(lat1,lon1) - y_offset;

  x_meas = 0;
  y_meas = 0;
  z_meas = bmp.readAltitude(air_press) - z_bias;

  a1 = (IMU.getAccelZ_mss()-a0_bias)+G;
  z_dot_meas = (a1 - a0)/Ts;
  
  a0 = a1;
}

void attitude_estimator() {
  phi = (tau*(phi+Ts*phi_dot_z) + Ts*phi_z)/(tau+Ts);
  theta = (tau*(theta+Ts*theta_dot_z) + Ts*theta_z)/(tau+Ts);
  psi = (tau*(psi+Ts*psi_dot_z) + Ts*psi_z)/(tau+Ts);

  phi_dot = phi_dot_z;
  theta_dot = theta_dot_z;
  psi_dot = psi_dot_z;

  Serial.print(psi);Serial.print(",");
  Serial.println(phi_z);
}


void position_estimator() {
//  x = (tau*(x+Ts*x_dot_meas) + Ts*x_meas)/(tau+Ts);
//  y = (tau*(y+Ts*y_dot_meas) + Ts*y_meas)/(tau+Ts);

  x = 0;
  y = 0;

  
  z = (tau_z*(z+Ts*z_dot_meas) + Ts*z_meas)/(tau_z+Ts);

//  Serial.println(z,6);

  x_dot = 0;
  y_dot = 0;
  z_dot = z_dot_meas;
};

void read_GPS() {
  //  gpsSerial.begin(9600);
//  bool GPS_ACQUIRED = false;
//
//  Serial.print("Get GPS");
//  while(!GPS_ACQUIRED) {
//    Serial.print(".");
//    while (gpsSerial.available() > 0) {
//      if (gps.encode(gpsSerial.read()))
//         if (gps.location.isValid()) {
//            double lat1 = gps.location.lat();
//            double lon1 = gps.location.lng();
//
//            x_offset = getX(lat1,lon1);
//            y_offset = getY(lat1,lon1);
//
//            GPS_ACQUIRED = true;
//          }
//          else {
////            Serial.println("Location: Not Available");
//          }
//      } 
//      delay(50);
//  }
//  Serial.println("GPS Fin");  
}

double getX(double lat1,double lon1) { 
  return RADIUS*cos(PIE/180 *lat1)*cos(PIE/180 *lon1);
}

double getY(double lat1,double lon1) { 
  return RADIUS*cos(PIE/180 *lat1)*sin(PIE/180 *lon1);
}
