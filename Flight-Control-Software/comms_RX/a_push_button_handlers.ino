void handle_power_button() {
  int reading = digitalRead(button1pin);
  if (reading != powerLastButtonState) {
    powerLastButtonState = millis();
  }

  if ((millis() - powerLastDebounceTime) > debounceDelay) {
    if (reading != powerButtonState) {
      powerButtonState = reading;
      if (powerButtonState == HIGH) {
        POWER_COMMAND = !POWER_COMMAND;
      }
    }
  }
  powerLastButtonState = reading;
}


void handle_manual_button() {
  int reading = digitalRead(button2pin);
  if (reading != manualLastButtonState) {
    manualLastDebounceTime = millis();
  }

  if ((millis() - manualLastDebounceTime) > debounceDelay) {
    if (reading != manualButtonState) {
      manualButtonState = reading;
      if (manualButtonState == HIGH) {
         MANUAL_CONTROL_COMMAND = !MANUAL_CONTROL_COMMAND;
      }
    }
  }
  manualLastButtonState = reading;
}

void readPotentometer() {
  potValue = analogRead(A0);
  potValue = map(potValue, 0, 1023, 0, 180);
}
