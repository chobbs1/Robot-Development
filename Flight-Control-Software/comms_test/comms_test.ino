#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte telemetry_pipe[6] = "00001";
const byte command_pipe[6] = "00002"; 

struct data {
  double x,y,z;
  double x_dot,y_dot,z_dot;
  double phi,theta,psi;
  double phi_dot,theta_dot,psi_dot;
  double t=0;
};

struct data TX_packet;

void setup() {
  Serial.begin(9600);
  
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  update_data_packet();
  Serial.println(sizeof(TX_packet));
//  Serial.println(TX_packet.t);
  radio.write(&TX_packet, sizeof(TX_packet));
  delay(100);
}

void update_data_packet() {
  TX_packet.t += 1;
  TX_packet.x = 1;
  TX_packet.y = 2;
  TX_packet.z = 3;
  TX_packet.phi = 4;
  TX_packet.theta = 5;
  TX_packet.psi = 6;
  TX_packet.x_dot = 7;
  TX_packet.y_dot = 8;
  TX_packet.z_dot = 9;
  TX_packet.phi_dot = 10;
  TX_packet.theta_dot = 11;
  TX_packet.psi_dot = 12;
}






