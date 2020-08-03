#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
const int button1pin = 2;
//boolean MOTOR_TEST_FLAG = false;

int potValue; 
int MOTOR_TEST_FLAG = HIGH;      
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;  
unsigned long debounceDelay = 50;

struct telemtry_t {
  float x,y,z;
  float phi,theta,psi;
  float t;
};

struct command_t {
  int MOTOR_TEST_FLAG;
  int MOTOR_SPEED_COMMAND;
};

struct telemtry_t Telemetry_RX;
struct command_t Command_TX;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

//  attachInterrupt(digitalPinToInterrupt(button1pin),buttonPressed1,RISING); 
  
}

void loop() {
  if (radio.available()) {
    radio.read(&Telemetry_RX, sizeof(telemtry_t));
    print_data(); 
  }

  
  handle_button1_press();
//  Serial.println(MOTOR_TEST_FLAG);
  Command_TX.MOTOR_TEST_FLAG = MOTOR_TEST_FLAG;

  readPotentometer();
  Serial.println(potValue);
  Command_TX.MOTOR_SPEED_COMMAND = potValue;
}

void readPotentometer() {
  potValue = analogRead(A0);
  potValue = map(potValue, 0, 1023, 0, 180);
}

void handle_button1_press() {
  int reading = digitalRead(button1pin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        MOTOR_TEST_FLAG = !MOTOR_TEST_FLAG;
      }
    }
  }
  lastButtonState = reading;
}

void print_data() {
  Serial.print(Telemetry_RX.t,0);Serial.print(",");
  Serial.print(Telemetry_RX.x,6);Serial.print(",");
  Serial.print(Telemetry_RX.y,6);Serial.print(",");
  Serial.print(Telemetry_RX.z,6);Serial.print(",");
  Serial.print(Telemetry_RX.phi,6);Serial.print(",");
  Serial.print(Telemetry_RX.theta,6);Serial.print(",");
  Serial.println(Telemetry_RX.psi,6);
}




