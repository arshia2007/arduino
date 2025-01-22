#include <Encoder.h>
#include <IntervalTimer.h> 

Encoder myEnc1(9, 8);
Encoder myEnc2(9, 8);

// Motor driver pins
#define motor1PWM 5
#define motor1DIR 4
#define motor2PWM 6
#define motor2DIR 7

//pid constants
float kp = 0.0;
float ki = 0.0;
float kd = 0.0; 

double prevError1 = 0, prevError2 = 0;          // double error1, error2, prevError1 = 0, prevError2 = 0;
long sp = 0;
float integ1=0.0, integ2=0.0;
float der1=0.0, der2=0.0;
float pid1 = 0.0, pid2 = 0.0;

IntervalTimer timer;

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Motor control pins setup
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1DIR, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2DIR, OUTPUT);

  // Initialize motors to stop
  analogWrite(motor1PWM, 0);
  digitalWrite(motor1DIR, LOW);
  analogWrite(motor2PWM, 0);
  digitalWrite(motor2DIR, LOW);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  timer.begin(calcPID, 75000);

}

void calcPID() {

  if (Serial.available()) {
    sp = Serial.parseInt();  
    sp = (sp / 360.0) * 1300;  
  }

  // Print encoder values for debugging
  // Serial.print("Enc1: "); Serial.print(myEnc1.read());
  // Serial.print(" Enc2: "); Serial.print(myEnc2.read());
  // Serial.print(" Target: "); Serial.println(sp);

  long currentCounts1 = myEnc1.read();
  long currentCounts2 = myEnc2.read();

  // PID Control (for M1)
  float err1 = sp - currentCounts1;
  integ1 = integ1 + (err1*0.075);
  der1 = (err1 - prevError1)/0.075;

  pid1 = (kp*err1) + (ki*integ1) + (kd*der1);
  prevError1 = err1;

  // PID Control (for M2)
  float err2 = sp - currentCounts2;
  integ2 = integ2 + (err2*0.075);
  der1 = (err2 - prevError2)/0.075;

  pid2 = (kp*err2) + (ki*integ2) + (kd*der2);
  prevError2 = err2;

  runMotor(motor1PWM,motor1DIR,pid1);
  runMotor(motor2PWM,motor2DIR,pid2);

}

void runMotor(int motorPWM, int motorDir, float speed) {
  int pwm = abs(speed);
  pwm = constrain(speed, 0, 16383);
  if (speed > 0) {      //to check direction: if +ve - HIGH, else LOW
    digitalWrite(motorDir, HIGH);
  } else if (speed < 0) {
    digitalWrite(motorDir, LOW);
    speed = -speed;
  } else {
    pwm = 0;
  }
  analogWrite(motorPWM, pwm);
}