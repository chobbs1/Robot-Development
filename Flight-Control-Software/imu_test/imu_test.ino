#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

MPU9250 IMU(Wire,0x68);
Adafruit_BMP280 bmp;
int status;
byte AK8963_Address = 0x68;

double phi_z,theta_z,psi_z;
double phi_dot_z,theta_dot_z,psi_dot_z;

void setup() { 
 Serial.begin (9600);
 Serial.println("Begin");

 status = IMU.begin();

  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
} 

void loop() { 
  IMU.readSensor();
  
  phi_z = atan2(IMU.getAccelY_mss(), sqrt(IMU.getAccelX_mss()*IMU.getAccelX_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
  theta_z = atan2(IMU.getAccelX_mss(), sqrt(IMU.getAccelY_mss()*IMU.getAccelY_mss() + IMU.getAccelZ_mss()*IMU.getAccelZ_mss()));
//  psi_z = atan2(IMU.getMagY_uT(),IMU.getMagX_uT());
   
   phi_dot_z = IMU.getGyroX_rads();
  theta_dot_z = IMU.getGyroY_rads();
  psi_dot_z = IMU.getGyroZ_rads();

  Serial.println(phi_z);

}

 
