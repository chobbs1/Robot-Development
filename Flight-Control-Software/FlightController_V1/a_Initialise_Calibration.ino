/*----------------Void Setup Functions--------------------------------------------------*/
// calibration values
double z_bias;
double phi_bias,theta_bias,psi_bias;
double phi_d_bias,theta_d_bias,psi_d_bias;
double a0_bias;

double phi_z,theta_z,psi_z;
double x_meas = 0,y_meas,z_meas=0;
double phi_dot_z,theta_dot_z,psi_dot_z;

double a0=0,a1; 

void initialise_sensors() {
//  Serial.println("Initialising sensors...");
  status = IMU.begin();

  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
                  
//  gpsSerial.begin(GPSBaud);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);  
  radio.openReadingPipe(1, command_pipe);
  radio.openWritingPipe(telemetry_pipe); 
  radio.setPALevel(RF24_PA_MIN);
  
}



void calibrate_sensors() {
//  Serial.println("Calibrating sensors...");
  
  double num_samples = 20;
  double phi_sum=0,theta_sum=0,psi_sum=0,z_sum = 0;
  double phi_d_sum=0,theta_d_sum=0,psi_d_sum=0;
  double a0_sum=0;
  
  for (int i=0;i<num_samples;i++) {
    read_sensors();

    phi_sum += phi_z;
    theta_sum += theta_z;
    psi_sum += psi_z;
    
    z_sum += z_meas;

    phi_d_sum += phi_dot_z;
    theta_d_sum += theta_dot_z;
    psi_d_sum += psi_dot_z;
    a0_sum = IMU.getAccelZ_mss();
  }

  phi_bias = phi_sum/num_samples;
  theta_bias = theta_sum/num_samples;
  psi_bias = psi_sum/num_samples;
  
  z_bias = z_sum/num_samples;

  phi_d_bias = phi_d_sum/num_samples;
  theta_d_bias = theta_d_sum/num_samples;
  psi_d_bias = psi_d_sum/num_samples;

  a0_bias = a0_sum/num_samples;

  

//  Serial.print("Phi bias = ");Serial.print(180/PIE*phi_bias,2);
//  Serial.print(" | Theta bias = ");Serial.print(180/PIE*theta_bias,2);
//  Serial.print(" | z bias = ");Serial.println(z_bias,2);
//
//  Serial.print("Phi_d bias = ");Serial.print(180/PIE*phi_d_bias,2);
//  Serial.print(" | Theta_d bias = ");Serial.print(180/PIE*theta_d_bias,2);
//  Serial.print(" | Psi_d bias = ");Serial.println(180/PIE*psi_d_bias,2);
  

  if(MAG_CAL_FLAG) {
    recalibrate_mag();
  } else {
    set_mag();
  }

  //  _GPS()
//  x_offset = getX(lat1,lon1);
//  y_offset = getY(lat1,lon1);
  
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
  IMU.setMagCalX(15.30,1.34);
  IMU.setMagCalY(-8.62,2.10);
  IMU.setMagCalZ(-41.97,0.56);  
}

void setup_motors() {
//  Serial.println("Motor Setup...");
  M1.attach(M1_PIN, MIN_THROTTLE, MAX_THROTTLE);
  M2.attach(M2_PIN, MIN_THROTTLE, MAX_THROTTLE);
  M3.attach(M3_PIN, MIN_THROTTLE, MAX_THROTTLE);
  M4.attach(M4_PIN, MIN_THROTTLE, MAX_THROTTLE);
}
