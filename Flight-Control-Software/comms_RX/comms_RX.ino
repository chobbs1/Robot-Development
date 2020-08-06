#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte telemetry_pipe[6] = "00001";
const byte command_pipe[6] = "00002"; 

const int button1pin = 2;
const int button2pin = 3;

int potValue; 
     
int powerButtonState = LOW;
int manualButtonState = LOW;
int powerLastButtonState = LOW;
int manualLastButtonState = LOW;

int MANUAL_CONTROL_COMMAND;
int MOTOR_SPEED_COMMAND;
int POWER_COMMAND;

unsigned long lastTime = 0;
unsigned long powerLastDebounceTime = 0;
unsigned long manualLastDebounceTime = 0;  
unsigned long debounceDelay = 50;

struct telemtry_t {
  float x,y,z;
  float phi,theta,psi;
  float t;
};

struct telemtry_rate_t {
  float x_dot,y_dot,z_dot;
  float phi_dot,theta_dot,psi_dot;
};

struct command_t {
  int MANUAL_CONTROL_COMMAND;
  int MOTOR_SPEED_COMMAND;
  int POWER_COMMAND;
};

struct telemtry_t Telemetry_RX;
struct telemtry_rate_t Telemetry_rate_RX;
struct command_t Command_TX;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);
  radio.openReadingPipe(1, telemetry_pipe);
  radio.openWritingPipe(command_pipe);
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  RX_telemetry();
  print_RX_telemetry(); 

  handle_power_button();
  handle_manual_button();
  readPotentometer();

  TX_commands();
//  print_TX_commands();
}










