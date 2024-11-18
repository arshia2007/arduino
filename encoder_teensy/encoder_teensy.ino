#include <Encoder.h>
#include "USBHost_t36.h"
// Encoder pins
#define encoderA 31  // Encoder Channel A
#define encoderB 30  // Encoder Channel B


//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
//BluetoothController bluet(myusb);   // version assumes it already was paired


//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  

// Motor control pins
#define motorPWM 5  // PWM pin for motor speed
#define motorDir 4   // Direction control pin

// Create Encoder object
Encoder myEnc(encoderA, encoderB);

unsigned long lastTime = 0;   // Last time RPM was calculated
long lastPosition = 0;        // Last encoder position
float currentRPM = 0.0;       // Calculated RPM

//const int pulsesPerRevolution = 325; // Set based on your encoder
const unsigned long interval = 75;   // Time interval in milliseconds
//const int gearRatio = 10;            // Planetary motor gear ratio

void setup() {
  Serial.begin(9600);

  // Motor control pins setup
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);

  // Initialize motor to stop
  analogWrite(motorPWM, 0);
  digitalWrite(motorDir, LOW);

  
  myusb.begin();
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();

  myusb.Task(); 
  if (joystick.available()) {

    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick.getAxis(0);
    leftX = map(leftStickX, 0, 255, -127, 127);
    

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick.getAxis(2);
    x = map(rightStickX, 0, 255, -127, 127);
    int rightStickY = joystick.getAxis(5);
    y = map(rightStickY, 0, 255, -127, 127);

    //to ignore small joystick values
    if (abs(x) < 5) x = 0;
    if (abs(y) < 5) y = 0;
    if (abs(leftX) < 5) leftX = 0;
  }
  else{
    Serial.print("Joystick Not Found");
  } 


  // Calculate RPM every interval milliseconds
  if (currentTime - lastTime >= interval) {
    long currentPosition = myEnc.read();               // Read current encoder position
    long positionChange = currentPosition - lastPosition; // Calculate position change
    lastPosition = currentPosition;                    // Update last position

    // Calculate RPM considering gear ratio
    currentRPM = (positionChange / 1300.0)*(60*(1000.0/interval)) ;

    // Print RPM
    Serial.print("RPM: ");
    Serial.println(currentRPM);

    lastTime = currentTime; // Update the last calculation time
  }

  runMotor1(y);
}
void runMotor1(float speed) {
  int pwmValue = map(speed,-84.67,84.67, -255, 255);
  pwmValue = abs(pwmValue); 
  if (speed > 0) {      //to check direction: if +ve - HIGH, else LOW
    digitalWrite(motorDir, HIGH);
  } else if (speed < 0) {
    digitalWrite(motorDir, LOW);
    speed = -speed;
  } else {
    pwmValue = 0;
  }
  analogWrite(motorPWM, pwmValue);
}
