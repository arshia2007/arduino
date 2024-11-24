#include <Encoder.h>
#include <IntervalTimer.h> 
#include "USBHost_t36.h"

Encoder myEnc(28,27);

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX; 

int PWM = 7;
int DIR = 6;
volatile long encoderCounts = 0;  // Shared variable to track encoder counts
volatile long lastCount = 0;      
volatile double rpm = 0;          // Stores the calculated RPM
long positionChange;

//pid constants
float kp = 0.5;
float ki = 0.01;
float kd = 0.01;

float sp, pid, prev_err, integ, der;

IntervalTimer timer; // Timer object for periodic execution

void calculateRPM() {

  myusb.Task();   // Handle USB host tasks

  if (joystick.available()) {
    int rightStickY = joystick.getAxis(5);
    y = map(rightStickY, 0, 255, -127, 127);

    // Ignore small joystick values
    if (abs(y) < 5) y = 0;

    sp = map(y, -127, 127, -750, 750);  //mapping joystick to rpm range 
  } else {
    Serial.println("Joystick Not Found");
  }

  //Serial.print("sp: ");
  //Serial.println(sp);

  long currentCounts = myEnc.read();
  positionChange = currentCounts - lastCount;
  lastCount = currentCounts;

  // Calculate RPM
  rpm = (positionChange / 1300.0) * (60 * (1000.0 / 75));

  //Serial.print("rpm: ");
  //Serial.println(rpm);

  //PID Control
  float err = sp - rpm;
  //Serial.print("error: ");
  //Serial.println(err);
  integ += err*0.075;             //integ += (err-prev_err);
  der = (err-prev_err)/0.075;
  pid = (kp*err) + (ki*integ) + (kd*der);
  prev_err = err;

  pid = constrain(pid, -255, 255);
  //Serial.print("y: ");
  //Serial.println(y);

  int pwmValue = abs(pid); 
  if (pid > 0) {      //to check direction: if +ve - HIGH, else LOW
    digitalWrite(DIR, HIGH);
  } else if (pid < 0) {
    digitalWrite(DIR, LOW);
  } else {
    pwmValue = 0;
  }
  analogWrite(PWM, pwmValue);

  Serial.print("sp: ");
  Serial.println(sp);
  Serial.print("   rpm: ");
  Serial.println(rpm);
  //Serial.print("  pid: ");
  // Serial.println(pid);
  
}

void setup() {
  Serial.begin(9600);

  // Motor control pins setup
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  // Initialize motor to stop
  analogWrite(PWM, 0);
  digitalWrite(DIR, LOW);

  myusb.begin();
  delay(2000);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  timer.begin(calculateRPM, 75000);
}

void loop() {
  
}
