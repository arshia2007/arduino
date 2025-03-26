#include <PS2X_lib.h>  // PS2 controller library

PS2X ps2x;  // create PS2 controller object

// Motor driver pins
int IN1 = 7; // Motor 1
int IN2 = 9; // Motor 2
int IN3 = 8; // Motor 3

// PWM pins for speed control
int ENA = 3; // Motor 1 PWM pin
int ENB = 5; // Motor 2 PWM pin
int ENC = 6; // Motor 3 PWM pin



// Motor speed (initial value)
int baseSpeed = 150;

void setup() {
  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize PS2 controller
   int error = ps2x.config_gamepad(13,11,10,12, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
 
  if(error == 0){
   Serial.println("Found Controller, configured successful");
 }
   
  else if(error == 1)
   Serial.println("No controller found, check wiring.");
   
  else if(error == 2)
   Serial.println("Controller found but not accepting commands.");
   
  else if(error == 3)
   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  // Setting each motor at low
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

void loop() {
  
  ps2x.read_gamepad();  // Read PS2 controller inputs

  int xAxis = ps2x.Analog(PSS_RX) - 128;  // Right joystick X-axis (centered around 0)
  int yAxis = 128 - ps2x.Analog(PSS_RY);  // Right joystick Y-axis (centered around 0)
  
  // Scale joystick inputs to desired speed range
  float Vx = (float)xAxis / 128.0 * baseSpeed;  // Horizontal speed component
  float Vy = (float)yAxis / 128.0 * baseSpeed;  // Vertical speed component

  // Calculate wheel speeds based on inverse kinematics
  float V1 = (Vx * cos(0) + Vy * sin(0));            // cos(0°) and sin(0°)
  float V2 = (Vx * cos(2.0944) + Vy * sin(2.0944));  // cos(120°) and sin(120°)
  float V3 = (Vx * cos(4.1888) + Vy * sin(4.1888));  // cos(240°) and sin(240°)

  // Set motor speeds based on calculated velocities
  runMotor1(V1);
  runMotor2(V2);
  runMotor3(V3);

  delay(20);  // Small delay for stability
}

void runMotor1(float speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    speed = -speed;
  } else {
    analogWrite(ENA, 0);
  }
  int pwmValue = map(abs(speed), 0, 300, 0, 200);
  pwmValue = constrain(pwmValue, 0, 200); // Ensure pwmValue is between 0 and 200
  analogWrite(ENA, pwmValue);
}

void runMotor2(float speed) {
  if (speed > 0) {
    digitalWrite(IN2, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN2, LOW);
    speed = -speed;
  } else {
    analogWrite(ENB, 0);    
  }
  int pwmValue = map(abs(speed), 0, 300, 0, 200);
  pwmValue = constrain(pwmValue, 0, 200); // Ensure pwmValue is between 0 and 200
  analogWrite(ENB, pwmValue);
}

void runMotor3(float speed) {
  if (speed > 0) {
    digitalWrite(IN3, LOW);
  } else if (speed < 0) {
    digitalWrite(IN3, HIGH);
    speed = -speed;
  } else {
    analogWrite(ENC, 0);
  }
  int pwmValue = map(abs(speed), 0, 300, 0, 200);
  pwmValue = constrain(pwmValue, 0, 200); // Ensure pwmValue is between 0 and 200
  analogWrite(ENC, pwmValue);
  
}