
void TX_commands() {
  radio.stopListening();
  
  Command_TX.MANUAL_CONTROL_COMMAND = MANUAL_CONTROL_COMMAND;
  Command_TX.POWER_COMMAND = POWER_COMMAND;
  Command_TX.MOTOR_SPEED_COMMAND = potValue;

  radio.write(&Command_TX, sizeof(Command_TX));
}

void print_TX_commands() {
  Serial.print("Power: ");Serial.print(Command_TX.POWER_COMMAND);
  Serial.print(" | Manual Control: ");Serial.print(Command_TX.MANUAL_CONTROL_COMMAND);
  Serial.print(" | Motor Speed: ");Serial.print(100*Command_TX.MOTOR_SPEED_COMMAND/180);
  Serial.println("%");
}

