struct data {
  double x,y,z;
  double phi,theta,psi;
  double t;
};

struct data TX_packet;


void TX_flight_data() {
  TX_packet.x = x;
  TX_packet.y = y;
  TX_packet.z = z;
  TX_packet.phi = phi;
  TX_packet.theta = theta;
  TX_packet.psi = psi;
  TX_packet.t = loop_start;

  Serial.println(TX_packet.t);
  
  radio.write(&TX_packet, sizeof(TX_packet));
}

