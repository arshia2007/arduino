#include <Encoder.h>
#include <IntervalTimer.h> // Teensy's built-in timer library

Encoder myEnc(31, 30);
// Motor control pins
#define motorPWM 5  // PWM pin for motor speed
#define motorDir 4   // Direction control pin

long currentCounts;

volatile long encoderCounts = 0; // Shared variable to track encoder counts

IntervalTimer timer; // Timer object for periodic execution

void calculateCOUNT() {
  // Read the encoder counts
  currentCounts = myEnc.read();

  Serial.print("Count: ");
  Serial.println(currentCounts);

}

void setup() {
  Serial.begin(9600);
  // Motor control pins setup
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);

  digitalWrite(motorDir, HIGH);
  analogWrite(motorPWM, 127);

  // Set up the timer to call calculateRPM every 1 second (1000000 microseconds)
  timer.begin(calculateCOUNT, 75000);
}

void loop() {
}