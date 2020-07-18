#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <MatrixMath.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define GPS_FLAG 0
#define MAG_CAL_FLAG 0

#define RADIUS 6378.137 
#define PIE 3.14159
#define G 9.81

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define ACTUATORS 4
#define STATES 7
#define ROT 3
#define VEC 1

int RXPin = 2;
int TXPin = 3;

int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
MPU9250 IMU(Wire,0x68);
Adafruit_BMP280 bmp;
int status;
byte AK8963_Address = 0x68;

// misc parameters
long loop_start;
double Ts = 0.01;

// tunable parameters
double tau = 0.02;
double air_press = 1017.8;

// state estimates
double x,y,z;
double x_dot,y_dot,z_dot;
double phi,theta,psi;
double phi_dot,theta_dot,psi_dot;

// sensor measurements
double x_meas = 0,y_meas,z_meas;
double x_meas0,y_meas0,z_meas0;
double x_dot_meas = 0,y_dot_meas = 0,z_dot_meas = 0;
double phi_z,theta_z,psi_z;
double phi_dot_z,theta_dot_z,psi_dot_z;
double a0 = 9.81;

// calibration values
double x_bias,y_bias,z_bias=0;
double phi_bias=0,theta_bias=0,psi_bias=0;

// define matrices
mtx_type R[ROT][ROT];
mtx_type R_prime[ROT][ROT] = {
    {0,0,0},
    {0,0,0},
    {0,0,0}
};

mtx_type U[ACTUATORS][VEC];


/*-------------------MAIN FUNCTION-----------------------------*/
void setup() {
 Serial.begin(115200);
 initialise_sensors();
 calibrate_sensors();
 Serial.println("Main Loop Start");
}


void loop() {
  loop_start = millis();
//  print_state();

  
  read_sensors(); 
  attitude_estimator();
  position_estimator();
  
  
  update_rotation_mat();
  compute_control();

  while (millis() - loop_start< 1000*Ts) {}
}

/*----------------Void Setup Functions--------------------------------------------------*/
void initialise_sensors() {
  Serial.println("Initialising sensors...");
  status = IMU.begin();

  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
                  
  gpsSerial.begin(GPSBaud);
}



void calibrate_sensors() {
  Serial.println("Calibrating sensors...");
  
  int num_samples = 5;
  
  for (int i=0;i<num_samples;i++) {
    read_sensors();
    phi_bias += phi_z;
    theta_bias += theta_z;
    psi_bias += psi_z;
    z_bias += z_meas;
  }

  phi_bias /= num_samples;
  theta_bias /= num_samples;
  psi_bias /= num_samples;
  z_bias /= num_samples;

  Serial.print("Phi = ");Serial.print(180/PIE*phi,2);
  Serial.print(" | Theta = ");Serial.print(180/PIE*theta,2);
  Serial.print(" | Psi = ");Serial.print(180/PIE*psi,2);
  

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
  IMU.setMagCalX(243.78,0.46);
  IMU.setMagCalY(38.82,3.50);
  IMU.setMagCalZ(-5.31,1.85);  
}

/*----------------------Sensor Fusion Functions-----------------------------------------*/
void read_sensors() {  
  IMU.readSensor();
  
  phi_z = atan2(IMU.getAccelY_mss(), sqrt(IMU.getAccelX_mss()*IMU.getAccelX_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  theta_z = atan2(IMU.getAccelX_mss(), sqrt(IMU.getAccelY_mss()*IMU.getAccelY_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  psi_z = atan2(IMU.getMagY_uT(),IMU.getMagX_uT());
  
  phi_dot_z = IMU.getGyroX_rads();
  theta_dot_z = IMU.getGyroY_rads();
  psi_dot_z = IMU.getGyroZ_rads();

//  read_GPS();  
//  x_meas = getX(lat1,lon1) - x_offset;
//  y_meas = getY(lat1,lon1) - y_offset;

  x_meas = 0;
  y_meas = 0;
  z_meas = bmp.readAltitude(air_press) - z_bias;

  x_dot_meas = (x_meas-x_meas0)/Ts;
  y_dot_meas = (y_meas-y_meas0)/Ts;
  z_dot_meas = (z_meas-z_meas0)/Ts;

  x_meas0 = x_meas;
  y_meas0 = y_meas;
  z_meas0 = z_meas;

}

void attitude_estimator() {
  phi = (tau*(phi+Ts*phi_dot_z) + Ts*phi_z)/(tau+Ts) - phi_bias;
  theta = (tau*(theta+Ts*theta_dot_z) + Ts*theta_z)/(tau+Ts) - theta_bias;
  psi = (tau*(psi+Ts*psi_dot_z) + Ts*psi_z)/(tau+Ts) - psi_bias;
}


void position_estimator() {
  x = (tau*(x+Ts*x_dot_meas) + Ts*x_dot_meas)/(tau+Ts);
  y = (tau*(y+Ts*y_dot_meas) + Ts*y_dot_meas)/(tau+Ts);
  z = (tau*(z+Ts*z_dot_meas) + Ts*z_dot_meas)/(tau+Ts);
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




/*-------------------Controller Functions------------------------------------------*/










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
  U[0][0] = 1;
  U[1][0] = 1;
  U[2][0] = 1;
  U[3][0] = 1;
}



/*---------------Miscellaneous Functions------------------------------*/

void print_state() {
  Serial.print("Time = ");Serial.print(loop_start);
  Serial.print(" | Phi = ");Serial.print(180/PIE*phi,2);
  Serial.print(" | Theta = ");Serial.print(180/PIE*theta,2);
  Serial.print(" | Psi = ");Serial.print(180/PIE*psi,2);

  Serial.print(" | x = ");Serial.print(x,2);
  Serial.print(" | y = ");Serial.print(y,2);
  Serial.print(" | z = ");Serial.println(z,2);
}




