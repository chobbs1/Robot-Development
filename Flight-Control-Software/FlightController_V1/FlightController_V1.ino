#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <MatrixMath.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define GPS_FLAG 0
#define MAG_CAL_FLAG 0

#define M1_PIN 6
#define M2_PIN 9
#define M3_PIN 11
#define M4_PIN 5
#define MAX_THROTTLE 2000
#define MIN_THROTTLE 1000

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
int potValue; 

int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
MPU9250 IMU(Wire,0x68);
Servo M1,M2,M3,M4;
Adafruit_BMP280 bmp;
int status;
byte AK8963_Address = 0x68;

// misc parameters
long loop_start;
int setup_time;
double Ts = 0.01;

// tunable parameters
double L = 0.223;
double Kf = 0.0002;
double Km = 0.0002;
double mass = 0.414;

// state estimates
double x,y,z;
double x_dot,y_dot,z_dot;
double phi,theta,psi;
double phi_dot,theta_dot,psi_dot;

// define matrices
mtx_type R[ROT][ROT];
mtx_type R_prime[ROT][ROT] = {
    {0,0,0},
    {0,0,0},
    {0,0,0}
};


/*-------------------MAIN FUNCTION-----------------------------*/
void setup() {
 Serial.begin(115200);
 initialise_sensors();
 calibrate_sensors();
 setup_motors(); 
 setup_time= millis();
}


void loop() {
  loop_start = millis() - setup_time;
//  print_state();
//  log_state();
  log_state_simulation();

  read_sensors(); 
  attitude_estimator();
  position_estimator();


//  if(millis()<30000) {
//    test_motors();
//  } else {
//    update_rotation_mat();
//    compute_control();
//  }
  
  

  while (millis() - loop_start - setup_time< 1000*Ts) {}
}



