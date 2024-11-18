#include "USBHost_t36.h"


// Motor driver pins
int IN1 = 7; // Motor 1
int IN2 = 9; // Motor 2
int IN3 = 8; // Motor 3

// PWM pins for speed control
int ENA = 3; // Motor 1 PWM pin
int ENB = 5; // Motor 2 PWM pin
int ENC = 6; // Motor 3 PWM pin

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device


//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  

void setup() {
  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);

  //to check teensy
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  myusb.begin();
  delay(2000);

  // Setting each motor at low
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

void loop() {
  
  myusb.Task(); 
  if (joystick.available()) {

    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick.getAxis(0);
    leftX = map(leftStickX, 0, 255, -127, 127);
    

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick.getAxis(2);
    x = map(rightStickX, 0, 255, -127, 127);
    int rightStickY = joystick.getAxis(5);
    y = map(rightStickY, 0, 255, 127, -127);

    //to ignore small joystick values
    if (abs(x) < 5) x = 0;
    if (abs(y) < 5) y = 0;
    if (abs(leftX) < 5) leftX = 0;
  }
  else{
    Serial.print("Joystick Not Found");
  }

  // Calculate wheel speeds based on inverse kinematics
  int V1 = ((x) * (-0.67) + (y) * 0 + (leftX) * (0.33));        
  int V2 = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (0.33)); 
  int V3 = ((x) * (0.33) + (y) * (0.57) + (leftX) * (0.33)); 

  Serial.print("V1: ");
  Serial.println(map(V1,0,100,0,200));
  Serial.print("V2: ");
  Serial.println(V2);
  Serial.print("V3: ");
  Serial.println(V3);

  // Set motor speeds based on calculated velocities
  //runMotor1(V1);
  //runMotor2(V2);
  //runMotor3(V3);

  delay(200);  // Small delay for stability
}

void runMotor1(float speed) {
  int pwmValue = map(abs(speed), 0, 100, 0, 200);
  pwmValue = constrain(pwmValue, 0, 200); // Ensure pwmValue is between 0 and 200
  if (speed > 0) {      //to check direction: if +ve - HIGH, else LOW
    digitalWrite(IN1, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    speed = -speed;
  } else {
    pwmValue = 0;
  }
  analogWrite(ENA, pwmValue);
}

void runMotor2(float speed) {
  int pwmValue = map(abs(speed), 0, 100, 0, 200);
  pwmValue = constrain(pwmValue, 0, 200); // Ensure pwmValue is between 0 and 200
  if (speed > 0) {
    digitalWrite(IN2, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN2, LOW);
    speed = -speed;
  } else {
    pwmValue = 0;    
  }
  analogWrite(ENB, pwmValue);
}

void runMotor3(float speed) {
  int pwmValue = map(abs(speed), 0, 100, 0, 200);
  pwmValue = constrain(pwmValue, 0, 200); // Ensure pwmValue is between 0 and 200
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    speed = -speed;
  } else {
    pwmValue = 0;
  }
  analogWrite(ENC, pwmValue);
}