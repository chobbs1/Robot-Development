
void RX_telemetryta() {
  radio.startListening();
  if (radio.available()) {
    radio.read(&Telemetry_RX, sizeof(telemtry_t));
  }
//  radio.stopListening();
}

void print_RX_telemetry() {
//    Serial.print(Telemetry_RX.t,0);Serial.print(",");
//    Serial.print(Telemetry_RX.x,6);Serial.print(",");
//    Serial.print(Telemetry_RX.y,6);Serial.print(",");
//    Serial.print(Telemetry_RX.z,6);Serial.print(",");
    Serial.print(Telemetry_RX.phi,6);Serial.print(",");
//    Serial.print(Telemetry_RX.theta,6);Serial.print(",");
//    Serial.println(Telemetry_RX.psi,6);
}

