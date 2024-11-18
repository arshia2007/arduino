#include <PS2X_lib.h>
PS2X ps2x;  // Create PS2 Controller Class object

void setup() {
  Serial.begin(9600);

  // Initialize the PS2 controller, configure the pins and check for connection errors
  int error = ps2x.config_gamepad(13,11,10,12,true,true);

  if (error == 0) {
    Serial.println("PS2 Controller Connected successfully!");
  } 
  else if (error == 1) {
    Serial.println("No controller found, check wiring.");
  }
  else if (error == 2) {
    Serial.println("Controller found but not accepting commands.");
  }
  else if (error == 3) {
    Serial.println("Controller refusing to enter Analog mode.");
  }

  delay(1000);  // Wait for a second before starting the loop
}

void loop() {

}
