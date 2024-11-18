#include <Encoder.h>
#include <IntervalTimer.h> // Teensy's built-in timer library
#include "USBHost_t36.h"

long prevT = 0;
float eprev = 0;
float eintegral = 0;

Encoder myEnc(31, 30);

int pos;

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  

// Motor control pins
#define motorPWM 5  // PWM pin for motor speed
#define motorDir 4   // Direction control pin

volatile double rpm = 0;         // Stores the calculated RPM

IntervalTimer timer; // Timer object for periodic execution
long lastPosition = 0;    

void calculateRPM() {
  // Read the encoder counts
  long currentPosition = myEnc.read();
  pos = currentPosition;
  long positionChange = currentPosition - lastPosition;
  lastPosition = currentPosition;  

  // Calculate RPM
  rpm= (positionChange / 1300.0)*(60*(1000.0/75)) ;
  
  // Debugging: Optional Serial output (use sparingly in interrupts)
  // Serial.print("RPM: ");
  // Serial.println(rpm);
}

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

  // Set up the timer to call calculateRPM every 1 second (1000000 microseconds)
  timer.begin(calculateRPM, 75000);
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
    y = map(rightStickY, 0, 255, -127, 127);

    //to ignore small joystick values
    if (abs(x) < 5) x = 0;
    if (abs(y) < 5) y = 0;
    if (abs(leftX) < 5) leftX = 0;
  }
  else{
    Serial.print("Joystick Not Found");
  } 

  int target=150;
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // PID constants
  float kp = 1.0;
  float kd = 0.0;
  float ki = 0.0;
  // error
  int e = target - rpm;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;
  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // store previous error
  eprev = e;
  Serial.print(target);
  Serial.print(" ");
  Serial.print(rpm);
  Serial.println();


  
  // Main loop can now run other tasks
  //Serial.print("Current RPM: ");
  //Serial.println(rpm);

  //delay(500); // Add some delay for smoother output (not required for other tasks)

  runMotor1(pwr);
}


void runMotor1(float speed) {
  int pwmValue = map(abs(speed), 0, 255, 0, 255);
  if (speed > 0) { 
    digitalWrite(motorDir, HIGH);
  } else if (speed < 0) {
    digitalWrite(motorDir, LOW);
  } else {
    pwmValue = 0; // Ensure motor stops when speed is 0
  }
  analogWrite(motorPWM, pwmValue);
}

