#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

struct data {
  float x,y,z;
  float x_dot,y_dot,z_dot;
  float phi,theta,psi;
  float phi_dot,theta_dot,psi_dot;
  float t;
};

struct data RX_packet;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

int x = 0;
int y = 0;
int z = 2;
int phi = 0;
int theta = 0;
float psi = 0;

void loop() {
  if (radio.available()) {
    
    radio.read(&RX_packet, sizeof(data));
    Serial.println(RX_packet.psi_dot);
  }

  psi += 0.01;

  Serial.print(x);Serial.print(",");
  Serial.print(y);Serial.print(",");
  Serial.print(z);Serial.print(",");
  Serial.print(phi);Serial.print(",");
  Serial.print(theta);Serial.print(",");
  Serial.println(psi);
  delay(10);
}
