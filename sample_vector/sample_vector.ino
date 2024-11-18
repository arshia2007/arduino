#include <PS2X_lib.h>  // PS2 controller library

PS2X ps2x;  // create PS2 controller object

// Motor driver pins 

//Digital pins for direction control
int IN1 = 2, IN2 = 3; // Motor 1
int IN3 = 4, IN4 = 5; // Motor 2
int IN5 = 6, IN6 = 7; // Motor 3

// PWM pins for speed control
int ENA = 9; // Motor 1 PWM pin
int ENB = 10; // Motor 2 PWM pin
int ENC = 11; // Motor 3 PWM pin

// Motor speed (0-255)
int speed = 150;

// PS2 Controller buttons mapping
#define UP_BUTTON 0x10
#define DOWN_BUTTON 0x40
#define LEFT_BUTTON 0x80
#define RIGHT_BUTTON 0x20

void setup() {
  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize PS2 controller
  int error = ps2x.config_gamepad(13, 11, 10, 12, true, true); // Setup pins for PS2 communication
  if (error == 0) {
    Serial.println("PS2 controller connected.");
  } else {
    Serial.println("PS2 controller connection failed.");
  }
}

void loop() {
  ps2x.read_gamepad(false, 0);  // Read PS2 controller inputs

  int xAxis = ps2x.Analog(PSS_RX);  // Right joystick X-axis
  int yAxis = ps2x.Analog(PSS_RY);  // Right joystick Y-axis
  
  // Speed control (optional)
  if (ps2x.Button(PSB_L1)) {
    speed += 10;
    if (speed > 255) speed = 255;
    delay(100);
  }
  if (ps2x.Button(PSB_R1)) {
    speed -= 10;
    if (speed < 0) speed = 0;
    delay(100);
  }

  // Map joystick inputs to motor movements
  if (xAxis < 100 && yAxis < 100) {  // Move left and forward (diagonal)
    moveLeftForward();
  } else if (xAxis < 100 && yAxis > 150) {  // Move left and backward (diagonal)
    moveLeftBackward();
  } else if (xAxis > 150 && yAxis < 100) {  // Move right and forward (diagonal)
    moveRightForward();
  } else if (xAxis > 150 && yAxis > 150) {  // Move right and backward (diagonal)
    moveRightBackward();
  } else if (xAxis < 100) {  // Move left
    moveLeft();
  } else if (xAxis > 150) {  // Move right
    moveRight();
  } else if (yAxis < 100) {  // Move forward
    moveForward();
  } else if (yAxis > 150) {  // Move backward
    moveBackward();
  } else {
    stopMotors();  // Stop if no directional input
  }

  delay(20);  // Small delay for stability
}

// Existing movement functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  analogWrite(ENA, 0);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
}

void moveLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
}

void moveRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

// New diagonal movement functions
void moveLeftForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  analogWrite(ENA, speed*0.73);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
}

void moveLeftBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  analogWrite(ENA, speed*0.73);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
}

void moveRightForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  analogWrite(ENA, speed*0.73);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
}

void moveRightBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  analogWrite(ENA, speed*0.73);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
}

