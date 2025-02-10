#include <Encoder.h>
#include <IntervalTimer.h> 

Encoder myEnc1(40, 41); // Encoder for Motor 1

// Motor driver pins
int motor1PWM = 23;
int motor1DIR = 21;

// PID constants
float kp = 5; 
float ki = 0.2;
float kd = 0.1; 

// PID variables
double prevError1 = 0;    
float integ1 = 0.0;       
float der1 = 0.0;         
float pid1 = 0.0;       

double setpoint = 0;      
// int sp = 0;               
double startPosition = 0; 
double endPosition = 0;  
double easeDuration = 2000;     // 2 seconds
unsigned long easeStartTime = 0; 

IntervalTimer timer;     

// Function to parse serial input for PID tuning and setpoint
void parseInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); 

    if (input.length() >= 12) { 
      kp = input.substring(0, 3).toFloat();
      ki = input.substring(3, 6).toFloat();
      kd = input.substring(6, 9).toFloat();
      endPosition = input.substring(9).toInt() * 1300 / 360.0; 
      // endPosition = sp;
      easeStartTime = millis(); 
    }
  }
}

// EaseInOutQuint Function (Easing curve)
float easeInOutQuint(float x) {
  return (x < 0.5) ? 16 * x * x * x * x * x : 1 - pow(-2 * x + 2, 5) / 2;
}

// PID calculation function
void calcPID() {
  // Read encoder position
  long currentCounts1 = myEnc1.read();

  // Calculate eased setpoint
  unsigned long currentTime = millis();
  double elapsedTime = currentTime - easeStartTime;

  if (elapsedTime < easeDuration) {
    float t = elapsedTime / easeDuration; 
    setpoint = startPosition + (endPosition - startPosition) * easeInOutQuint(t);
  } else {
    setpoint = endPosition; 
  }

  // PID calculations
  float err1 = setpoint - currentCounts1; 
  integ1 = integ1 + (err1 * 0.075);
  der1 = (err1 - prevError1) / 0.075;    

  pid1 = (kp * err1) + (ki * integ1) + (kd * der1); 
  prevError1 = err1;

  // Run motor with calculated PID output
  runMotor(motor1PWM, motor1DIR, pid1);

  // Debugging: Print setpoint, current position, and PID output
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" | Position: ");
  Serial.print(currentCounts1);
  Serial.print(" | PID Output: ");
  Serial.println(pid1);
}

// Function to run the motor with direction and speed
void runMotor(int motorPWM, int motorDir, float speed) {
  int pwm = abs(speed); 
  pwm = constrain(pwm, 0, 16383); 

  if (speed > 0) {
    digitalWrite(motorDir, HIGH); 
  } else if (speed < 0) {
    digitalWrite(motorDir, LOW); 
    speed = -speed; 
  } else {
    pwm = 0; 
  }

  analogWrite(motorPWM, pwm); 
}

void setup() {
  Serial.begin(9600); 

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Motor control pins setup
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1DIR, OUTPUT);

  // Initialize motor to stop
  analogWrite(motor1PWM, 0);
  digitalWrite(motor1DIR, LOW);

  analogWriteResolution(14);     
  analogWriteFrequency(0, 9000);

  // Initialize easing parameters
  startPosition = myEnc1.read();
  // endPosition = sp; 
  easeStartTime = millis();

  // Start PID timer
  timer.begin(calcPID, 75000); 
}

void loop() {
  parseInput(); 
}
