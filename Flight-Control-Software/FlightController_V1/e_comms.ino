struct data {
  double x,y,z;
  double phi,theta,psi;
  double t;
};

//struct command_t {
//  int MANUAL_CONTROL_COMMAND;
//  int MOTOR_SPEED_COMMAND;
//  int POWER_COMMAND;
//};

struct command_t ;
struct data TX_packet;

void TX_flight_data() {
  radio.stopListening();
  
  TX_packet.x = x;
  TX_packet.y = y;
  TX_packet.z = z;
  TX_packet.phi = phi;
  TX_packet.theta = theta;
  TX_packet.psi = psi;
  TX_packet.t = loop_start;

  radio.write(&TX_packet, sizeof(TX_packet));
  
}

void RX_commands() {
  radio.startListening();
  long startTimer = millis();
//  while(millis()-startTimer<0.05) {
  for(int i=0;i<20;i++) {
//  while(radio.available()) {
    if (radio.available()) {
      radio.read(&Command_RX, sizeof(Command_RX));
      Serial.println("Here");
      break;
    }
  }
//  }
//  radio.stopListening();
}

void print_RX_data() {
  Serial.print("Power: ");Serial.print(Command_RX.POWER_COMMAND);
  Serial.print(" | Manual Control: ");Serial.print(Command_RX.MANUAL_CONTROL_COMMAND);
  Serial.print(" | Motor Speed: ");Serial.print(100*Command_RX.MOTOR_SPEED_COMMAND/180);
  Serial.println("%");
}






