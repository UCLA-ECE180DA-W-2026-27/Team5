#include <Arduino.h>
#include "state_machine.h"

void readSerialCommands() {
  static String command;
  static bool overflow = false;
  for (int i = 0; i < 64 && Serial.available(); i++) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (!overflow) state_machine_command(command.c_str());
      else Serial.println("Command too long.");
      command = "";
      overflow = false;
    } else if (!overflow) {
      if (command.length() < 48) command += c;
      else overflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  state_machine_init();
}
void loop() {
  readSerialCommands();
  state_machine_update();
  delay(1);
}
