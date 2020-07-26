# include <Servo.h>

#define M1_PIN 11
#define M2_PIN 9
#define M3_PIN 6
#define M4_PIN 5
#define MAX_THROTTLE 2000
#define MIN_THROTTLE 1000

Servo M1,M2,M3,M4;
int potValue; 

void setup() { 
   M1.attach(M1_PIN, MIN_THROTTLE, MAX_THROTTLE);
   M2.attach(M2_PIN, MIN_THROTTLE, MAX_THROTTLE);
   M3.attach(M3_PIN, MIN_THROTTLE, MAX_THROTTLE);
   M4.attach(M4_PIN, MIN_THROTTLE, MAX_THROTTLE);
   
   Serial.begin (9600);
   Serial.println("Begin");
} 

void loop() { 
   potValue = analogRead(A3);
   potValue = map(potValue, 0, 1023, 0, 180);

   M1.write(potValue);
   M2.write(potValue);
   M3.write(potValue);
   M4.write(potValue);
   Serial.println(potValue);
}

 


//#include <MPU9250.h>
//
//
//MPU9250 IMU(Wire,0x68);
//int status;
//int potValue=180;
//int throttle;
//const int buff_len = 8;
//double x_acc_buff[buff_len],y_acc_buff[buff_len],z_acc_buff[buff_len],gyro_x_bff[buff_len];
//double angle_ref = 0;
//int bias = 50;
//double x,y,z,gyro_x,angle,error,command,right_command,left_command;
//int Kp,Ki,Kd;
//int state = 0, startTestPin = 12;
//
//void setup() {
//  Serial.begin(115200);
//  
//  RIGHT.attach(10, 1000, 2000);
//  status = IMU.begin();
//  initialise_buffers(x_acc_buff,y_acc_buff,z_acc_buff,gyro_x_bff);
//  Serial.println("angle,throttle,time");
//  millis();
//}
//
//void loop() {    
//  IMU.readSensor();
//  x = update_and_read_buffer(IMU.getAccelX_mss(),x_acc_buff);
//  y = update_and_read_buffer(IMU.getAccelY_mss(),y_acc_buff);
//  z = update_and_read_buffer(IMU.getAccelZ_mss(),z_acc_buff);
//  gyro_x = update_and_read_buffer(IMU.getGyroX_rads()*180/3.142,gyro_x_bff);
//  angle = get_angle(x,y,z);
//  
//  if(digitalRead(startTestPin)==0) {
//    Serial.print("MANUAL/CALIBRATION CONTROL: ");
//    potValue = analogRead(A0);
//    potValue = map(potValue, 0, 1023, 0, 180);
//    RIGHT.write(potValue);
////    LEFT.write(potValue);
//    Serial.print("PWM value = ");Serial.println(potValue);
//  } else {
//    Serial.print("AUTONOMOUS CONTROL: ");  
//
//    error = angle_ref - angle;
//    Kp = 2;
//    Kd = 2;
//    Ki = 0;
//  
//    command = (Kp * error - Kd * gyro_x)/10;
//    right_command = bias - command;
//    left_command = bias + command;
//    RIGHT.write(right_command);
//    LEFT.write(left_command );
//
//    Serial.print("time = ");Serial.print(millis());
//    Serial.print(",right = "); Serial.print(right_command);
//    Serial.print(",left = "); Serial.print(left_command);
//    Serial.print(",angle = "); Serial.print(angle);
//    Serial.print(",gyro = "); Serial.println(gyro_x);
//  }
//
//}
//
//double update_and_read_buffer(double newVal,double buff[]) {
//  double count = newVal;
//  for(int i=0;i<buff_len-1;i++) {
//    buff[i] = buff[i+1];
//    count += buff[i];
//  }
//  buff[buff_len-1] = newVal;
//  return count/buff_len;
//}
//
//void initialise_buffers(double accX[], double accY[], double accZ[],double gyro_x[]) {
//  for(int i=0;i<buff_len;i++) {
//    IMU.readSensor();
//    double x = IMU.getAccelX_mss();
//    double y = IMU.getAccelY_mss();
//    double z = IMU.getAccelZ_mss();
//    double x_dot = IMU.getGyroX_rads()*180/3.142;
//    accX[i] = x;
//    accY[i] = y;
//    accZ[i] = z;
//    gyro_x[i] = x_dot;
//  }
//}
//
//void print_buff(double buff[]) {
//  for(int i=0;i<buff_len;i++) {
//    Serial.print(buff[i]);Serial.print(" ");
//  }
//  Serial.println();
//}
//
//double get_angle(double X,double Y,double Z) { 
//  return 180/3.142*atan(Y/sqrt(X*X+Z*Z));
//}

