/*---------------Miscellaneous Functions------------------------------*/

void print_state() {
//  Serial.print("Time = ");Serial.println(loop_start);

//  Serial.print("Phi = ");
  Serial.println(180/PIE*phi,2);
//  Serial.print(" | Theta = ");
//  Serial.println(180/PIE*theta,2);
//  Serial.print(" | Psi = ");Serial.print(180/PIE*psi,2);
//
//  Serial.print(" | x = ");Serial.print(x,2);
//  Serial.print(" | y = ");Serial.print(y,2);
//  Serial.print(" | z = ");Serial.println(z,2);
//
//  Serial.print("Phi_dot = ");Serial.print(180/PIE*phi_dot,2);
//  Serial.print(" | Theta_dot = ");Serial.print(180/PIE*theta_dot,2);
//  Serial.print(" | Psi_dot = ");Serial.print(180/PIE*psi_dot,2);
//
//  Serial.print(" | x_dot = ");Serial.print(x_dot,2);
//  Serial.print(" | y_dot = ");Serial.print(y_dot,2);
//  Serial.print(" | z_dot = ");Serial.println(z_dot,2);

}


void log_state_simulation() {
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(z);
  Serial.print(",");
  Serial.print(phi,6);
  Serial.print(" ,");
  Serial.print(theta,6);
  Serial.print(",");
  Serial.println(psi,6);
}

void log_state() {
  Serial.print(loop_start);
  Serial.print(",");
  Serial.print(phi,6);
  Serial.print(" ,");
  Serial.print(180/PIE*theta,6);
  Serial.print(",");
  Serial.print(180/PIE*psi,6);
  Serial.print(",");
  Serial.println(z,6);
}
